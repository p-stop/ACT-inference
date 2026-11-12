"""Smoke test for the ACT policy server on this machine (same gRPC + pickle protocol as the robot client).

Sends real observations from a LeRobot dataset (images JPEG-compressed like the bimanual client does), receives the
action chunks, compares the first predicted action with the recorded action and reports round-trip times.

    .venv/bin/python remote_inference/smoke_client.py [--port 18151] [--dataset DIR] [--episode 0] [--n 20]
"""
import argparse, pickle, sys, time
from pathlib import Path

import cv2
import grpc
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/aug01"))
from common import Source, CAMS  # noqa: E402
from lerobot.async_inference.helpers import RemotePolicyConfig, TimedObservation, hw_to_dataset_features  # noqa: E402
from lerobot.transport import services_pb2, services_pb2_grpc  # noqa: E402
from lerobot.transport.utils import grpc_channel_options, send_bytes_in_chunks  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=18151)
ap.add_argument("--dataset", default="<PRO5000_ACT_DIR>/datasets/01_ACT")
ap.add_argument("--episode", type=int, default=0)
ap.add_argument("--n", type=int, default=20)
args = ap.parse_args()

src = Source(args.dataset)
names = src.info["features"]["observation.state"]["names"]
cams = {c.split(".")[-1]: c for c in CAMS}
hw = {n: float for n in names} | {k: (480, 640, 3) for k in cams}
features = hw_to_dataset_features(hw, "observation", use_video=False)

ch = grpc.insecure_channel(f"127.0.0.1:{args.port}", grpc_channel_options())
stub = services_pb2_grpc.AsyncInferenceStub(ch)
t = time.time(); stub.Ready(services_pb2.Empty()); print(f"Ready ok ({(time.time() - t) * 1000:.0f} ms)")
cfg = RemotePolicyConfig(policy_type="act", pretrained_name_or_path="(server decides)", lerobot_features=features,
                         actions_per_chunk=100, device="cuda")
t = time.time(); stub.SendPolicyInstructions(services_pb2.PolicySetup(data=pickle.dumps(cfg)))
print(f"policy loaded ({time.time() - t:.1f} s)")

rows = src.rows(args.episode); n = src.length(args.episode)
frames = np.linspace(30, n - 120, args.n).astype(int)
readers = {k: src.reader(args.episode, c) for k, c in cams.items()}
rtts, errs, sends, sizes = [], [], [], []
for step, f in enumerate(frames):
    raw = {nm: float(v) for nm, v in zip(names, src.state[rows][f])}
    for k, r in readers.items():
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(r.get(f), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        raw[k] = {"__codec__": "jpeg", "data": buf.tobytes()}
    obs = TimedObservation(timestamp=time.time(), timestep=int(f), observation=raw, must_go=True)
    payload = pickle.dumps(obs)
    t = time.time()
    stub.SendObservations(send_bytes_in_chunks(payload, services_pb2.Observation, silent=True))
    t_send = (time.time() - t) * 1000
    acts = []
    while not acts and time.time() - t < 5:
        msg = stub.GetActions(services_pb2.Empty())
        if msg.data:
            acts = pickle.loads(msg.data)
    rtt = (time.time() - t) * 1000
    if not acts:
        print(f"frame {f}: no actions within 5 s"); continue
    a0 = np.asarray(acts[0].get_action(), dtype=np.float32).reshape(-1)
    err = float(np.abs(a0 - src.action[rows][f]).max())
    rtts.append(rtt); errs.append(err)
    sends.append(t_send); sizes.append(len(payload))
    print(f"frame {f:4d}: {len(acts)} actions, first-action max |pred - recorded| = {err:.3f} rad, round trip {rtt:.0f} ms "
          f"(upload {len(payload) / 1024:.0f} KB in {t_send:.0f} ms)")
print(f"round trip median {np.median(rtts):.0f} ms, p90 {np.percentile(rtts, 90):.0f} ms, max {np.max(rtts):.0f} ms; "
      f"upload median {np.median(sends):.0f} ms for {np.median(sizes) / 1024:.0f} KB (~{np.median(sizes) / np.median(sends) / 1024:.2f} MB/s); "
      f"first-action error median {np.median(errs):.3f} rad")
