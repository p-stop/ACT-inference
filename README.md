# ACT-inference

Remote inference server for an ACT policy (LeRobot 0.6.1) driving a dual-arm Piper robot.
The policy runs on a GPU server and listens only on `127.0.0.1`; the robot machine connects through an SSH tunnel
and streams observations / receives action chunks with LeRobot's async-inference gRPC protocol.

| File | What it is |
|---|---|
| `act_policy_server.py` | Policy server: subclass of `lerobot.async_inference.PolicyServer`. Decodes JPEG-compressed images sent by the client, checks that the checkpoint's input features match the robot, warms the model up, optionally dumps observations for debugging. |
| `start_server.sh` | Launcher: picks the Python environment (a copy on a fast disk if present, otherwise the conda env `ACT`), syncs the checkpoint, starts the server on `127.0.0.1:${PORT:-18151}`, logs to `logs/`. |
| `pro5000/start_server.sh` | Launcher variant for a second GPU server with a uv `.venv`: picks a model by name (`ACT_bi`, `ACT_bi/240000`, ...), `--list` shows the available checkpoints, and picks the least busy GPU automatically. |
| `smoke_client.py` | Smoke test on the server itself: sends real observations from a LeRobot dataset with the same gRPC protocol as the robot client, compares the first predicted action with the recorded one and reports round-trip times (imports `common.py` from the training repo's `aug01/`). |
| `README_ACT远程推理.md` | Full setup notes (Chinese): architecture, the restricted tunnel key, step-by-step start-up, parameters, troubleshooting, removal. |

The robot-side client (`start_client.sh`, the `bi_piper_ros` robot plugin) lives on the robot machine and is not part of this repo.

## Placeholders to replace

Machine-specific details were replaced by placeholders. Edit them before use (in `start_server.sh` you can also set the
environment variables `ACT_DIR`, `CONDA_DIR`, `INFER_DIR`, `TRAIN_NAME` instead of editing the file).

| Placeholder | Meaning |
|---|---|
| `<WXR5_ACT_DIR>` | ACT working directory on the GPU server (contains `outputs/train/...` and `remote_inference/`) |
| `<WXR5_CONDA_DIR>` | miniconda install directory on the GPU server (with the conda env `ACT`) |
| `<WXR5_FAST_DIR>` | a directory on a fast (NVMe) disk for the optional environment / checkpoint copy |
| `<WXR5_HOME>` | home directory on the GPU server |
| `<WXR5_USER>`, `<WXR5_IP>` | login user and address of the GPU server |
| `<WXR5_HOSTKEY_FINGERPRINT>` | the GPU server's SSH host-key fingerprint (check it yourself with `ssh-keygen -lf`) |
| `<TUNNEL_KEY_PATH>`, `<TUNNEL_KEY_NAME>` | the dedicated SSH key on the robot machine that may only forward the inference port |
| `<ROBOT_HOME>`, `<ROBOT_PROJECT_DIR>`, `<ROBOT_USER>`, `<ROBOT_HOST>` | home / project directory / user / host name of the robot machine |
| `<HF_USER>` | Hugging Face user that owns the datasets |
| `<TRAIN_NAME>` | name of the training run whose last checkpoint `start_server.sh` uses by default |

## Quick start

```bash
# GPU server
GPU=0 bash start_server.sh <WXR5_ACT_DIR>/outputs/train/<TRAIN_NAME>/checkpoints/last/pretrained_model
# robot machine (see README_ACT远程推理.md for the tunnel and client)
bash start_client.sh --actions_per_chunk=100 --chunk_size_threshold=0.2 --aggregate_fn_name=latest_only
```

Security note: LeRobot 0.6.1's gRPC protocol uses pickle and has no authentication, which is why the server only listens on
localhost and is reached through an SSH tunnel with a key restricted to `permitopen="127.0.0.1:18151"`.
