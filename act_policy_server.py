"""ACT 远程推理服务端：lerobot 0.6.1 async_inference.PolicyServer 的薄包装（在 wxr5 上运行）。

相对原版只改了这几处，协议（gRPC + pickle）不变，原版 robot_client 也能连：
1. 观测里的图像如果是客户端压好的 JPEG（{"__codec__": "jpeg", "data": bytes}），先解码回 RGB uint8；
   原始 numpy 图像照常处理。双臂机器经 WiFi 只有约 9 MB/s，三路 480x640 原图每帧 2.8 MB 太慢。
2. 「观测和上次差不多就跳过推理」的阈值改成可配置的 --similarity_atol（弧度，默认 0 表示不跳过）。
   原版写死 atol=1，是按 so100 的「度」定的，用在弧度上等于几乎永远跳过，机械臂会一顿一顿地等。
3. --policy_path / --device 可以在服务端固定检查点和设备，客户端传来的值只作参考。
4. 加载后检查检查点的输入特征和机械臂的特征是否对得上，对不上直接报错；加载后做一次预热推理；
   同一个检查点在客户端重连时不重复加载。
5. 新客户端连上时把上一次处理过的观测也清掉。
"""

import logging
import pickle  # nosec
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat
from concurrent import futures

import cv2
import draccus
import grpc
import numpy as np
import torch

from lerobot.async_inference.configs import PolicyServerConfig
from lerobot.async_inference.helpers import RemotePolicyConfig, observations_similar
from lerobot.async_inference.policy_server import PolicyServer
from lerobot.policies import get_policy_class, make_pre_post_processors
from lerobot.transport import services_pb2, services_pb2_grpc  # type: ignore


@dataclass
class ActPolicyServerConfig(PolicyServerConfig):
    # 非空时忽略客户端传来的 pretrained_name_or_path，固定用这个检查点（pretrained_model 目录）
    policy_path: str = field(default="", metadata={"help": "Checkpoint dir on this server; overrides the client's"})
    # 非空时忽略客户端传来的 policy_device
    device: str = field(default="", metadata={"help": "Inference device; overrides the client's"})
    # 观测关节状态（弧度/米）与上次推理时的 L2 距离小于它就跳过；0 = 不跳过
    similarity_atol: float = field(default=0.0, metadata={"help": "Skip inference if state moved less than this"})
    warmup: bool = field(default=True, metadata={"help": "Run one dummy inference after loading"})
    # 调试：非空时把收到的观测（解码后的三路 RGB 图像 + 关节状态）存到这个目录，最多 dump_obs_count 帧，间隔 dump_obs_every_s 秒
    dump_obs_dir: str = field(default="", metadata={"help": "Save received observations (PNG + state) here"})
    dump_obs_count: int = field(default=5, metadata={"help": "How many observations to save"})
    dump_obs_every_s: float = field(default=3.0, metadata={"help": "Min seconds between saved observations"})


def decode_images(observation: dict) -> int:
    """把客户端压成 JPEG 的图像就地解码回 RGB uint8 (H, W, 3)，返回解码了几张。"""
    n = 0
    for key, value in list(observation.items()):
        if isinstance(value, dict) and value.get("__codec__") == "jpeg":
            bgr = cv2.imdecode(np.frombuffer(value["data"], dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError(f"JPEG decode failed for {key}")
            observation[key] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            n += 1
    return n


class ActPolicyServer(PolicyServer):
    def __init__(self, config: ActPolicyServerConfig):
        super().__init__(config)
        self._loaded_key = None

    def _reset_server(self) -> None:
        super()._reset_server()
        self.last_processed_obs = None

    def SendPolicyInstructions(self, request, context):  # noqa: N802
        if not self.running:
            self.logger.warning("Server is not running. Ignoring policy instructions.")
            return services_pb2.Empty()
        spec = pickle.loads(request.data)  # nosec  (only reachable through the SSH tunnel)
        if not isinstance(spec, RemotePolicyConfig):
            raise TypeError(f"Policy specs must be a RemotePolicyConfig. Got {type(spec)}")
        cfg = self.config
        if cfg.policy_path and spec.pretrained_name_or_path != cfg.policy_path:
            self.logger.info(f"Client asked for {spec.pretrained_name_or_path}; using server --policy_path {cfg.policy_path}")
            spec.pretrained_name_or_path = cfg.policy_path
        if cfg.device:
            spec.device = cfg.device

        key = (spec.policy_type, spec.pretrained_name_or_path, spec.device, tuple(sorted(spec.rename_map.items())))
        if key == self._loaded_key and self.policy is not None:
            self.logger.info(f"Policy already loaded from {spec.pretrained_name_or_path}; reusing it")
            self.lerobot_features = spec.lerobot_features
            self.actions_per_chunk = spec.actions_per_chunk
        else:
            super().SendPolicyInstructions(services_pb2.PolicySetup(data=pickle.dumps(spec)), context)
            self._loaded_key = key
        self._check_features(spec)
        if cfg.warmup:
            self._warmup()
        chunk = getattr(self.policy.config, "chunk_size", None)
        if chunk is not None and spec.actions_per_chunk > chunk:
            self.logger.warning(f"actions_per_chunk={spec.actions_per_chunk} > policy chunk_size={chunk}; "
                                f"chunks will only have {chunk} actions")
        return services_pb2.Empty()

    def _check_features(self, spec: RemotePolicyConfig) -> None:
        """检查点要的每个输入在客户端的特征里都有，形状一致（图像按 HWC 对比 CHW）。"""
        have = spec.lerobot_features
        problems = []
        for name, ft in self.policy.config.input_features.items():
            if name not in have:
                problems.append(f"missing {name} (robot provides: {sorted(have)})")
                continue
            shape = tuple(have[name]["shape"])
            want = tuple(ft.shape)
            if len(want) == 3 and len(shape) == 3:
                shape = (shape[2], shape[0], shape[1])  # HWC -> CHW
            if shape != want and len(want) != 3:
                problems.append(f"{name}: robot {shape} vs policy {want}")
            elif shape != want:
                self.logger.warning(f"{name}: robot image {shape} will be resized to {want}")
        out = self.policy.config.output_features.get("action")
        if out is not None and "action" in have and tuple(have["action"]["shape"]) != tuple(out.shape):
            problems.append(f"action: robot {have['action']['shape']} vs policy {out.shape}")
        if problems:
            msg = "Checkpoint does not match the robot: " + "; ".join(problems)
            self.logger.error(msg)
            raise ValueError(msg)
        state = have.get("observation.state", {}).get("names")
        self.logger.info(f"Features OK. observation.state order: {state}")

    @torch.no_grad()
    def _warmup(self) -> None:
        batch = {k: torch.zeros((1, *ft.shape), device=self.device) for k, ft in self.policy.config.input_features.items()}
        t = time.perf_counter()
        for _ in range(2):
            self.policy.predict_action_chunk(batch)
        if str(self.device).startswith("cuda"):
            torch.cuda.synchronize()
        self.logger.info(f"Warmup done ({(time.perf_counter() - t) * 1000:.0f} ms for 2 passes)")

    def _enqueue_observation(self, obs) -> bool:
        decode_images(obs.get_observation())
        self._maybe_dump(obs)
        return super()._enqueue_observation(obs)

    def _maybe_dump(self, obs) -> None:
        cfg = self.config
        if not cfg.dump_obs_dir:
            return
        n = getattr(self, "_dumped", 0)
        now = time.monotonic()
        if n >= cfg.dump_obs_count or now - getattr(self, "_last_dump", -1e9) < cfg.dump_obs_every_s:
            return
        self._dumped, self._last_dump = n + 1, now
        import json
        import os
        out = os.path.join(cfg.dump_obs_dir, f"obs_{n:02d}_t{obs.get_timestep():05d}")
        os.makedirs(out, exist_ok=True)
        state = {}
        for key, value in obs.get_observation().items():
            if isinstance(value, np.ndarray) and value.ndim == 3:
                cv2.imwrite(os.path.join(out, f"{key}.png"), cv2.cvtColor(value, cv2.COLOR_RGB2BGR))
            elif isinstance(value, (int, float, np.floating)):
                state[key] = float(value)
            elif isinstance(value, str):
                state[key] = value
        with open(os.path.join(out, "state.json"), "w") as f:
            json.dump(state, f, indent=1)
        self.logger.info(f"Saved observation to {out}")

    def _obs_sanity_checks(self, obs, previous_obs) -> bool:
        with self._predicted_timesteps_lock:
            predicted = obs.get_timestep() in self._predicted_timesteps
        if predicted:
            return False
        atol = self.config.similarity_atol
        if atol > 0 and observations_similar(obs, previous_obs, lerobot_features=self.lerobot_features, atol=atol):
            return False
        return True


@draccus.wrap()
def serve(cfg: ActPolicyServerConfig):
    logging.info(pformat(asdict(cfg)))
    policy_server = ActPolicyServer(cfg)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    services_pb2_grpc.add_AsyncInferenceServicer_to_server(policy_server, server)
    server.add_insecure_port(f"{cfg.host}:{cfg.port}")
    policy_server.logger.info(f"ACT PolicyServer started on {cfg.host}:{cfg.port} "
                              f"(policy_path={cfg.policy_path or '<from client>'}, device={cfg.device or '<from client>'})")
    server.start()
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        policy_server.logger.info("Ctrl+C: stopping server")
        server.stop(grace=1)
    policy_server.logger.info("Server terminated")


if __name__ == "__main__":
    serve()
