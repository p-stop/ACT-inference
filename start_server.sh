#!/usr/bin/env bash
# 在 wxr5 上启动 ACT 远程推理服务端（只监听 127.0.0.1，双臂机器经 SSH 隧道连进来）。
# 用法：bash start_server.sh [检查点目录] [其他 --参数=值 ...]
#   检查点目录默认是本次训练的最终检查点；环境变量 GPU（默认 0）选卡，PORT（默认 18151）选端口。
# 例：  bash start_server.sh
#       GPU=0 bash start_server.sh <WXR5_ACT_DIR>/outputs/train/xxx/checkpoints/020000/pretrained_model
#
# /data0 是一块很忙的机械盘，从它导入 torch、读检查点可能要好几分钟。所以：
#   - $INFER/env（ACT 环境的副本）和 $INFER/lerobot_src（lerobot 源码的副本）都在时，用它们（NVMe 盘）；
#   - 检查点每次启动前同步到 $INFER/ckpt/<训练名>_<步数>/（已复制过的只比对，不重复复制）；
#   - 副本不在时（例如已删除），自动退回 conda 环境 ACT 和 /data0 上的检查点。
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# ---- 按自己的机器修改（或用同名环境变量覆盖）----
ACT_DIR="${ACT_DIR:-<WXR5_ACT_DIR>}"            # 训练目录（里面有 outputs/train/...）
CONDA_DIR="${CONDA_DIR:-<WXR5_CONDA_DIR>}"      # miniconda 安装目录（有 conda 环境 ACT）
INFER="${INFER_DIR:-<WXR5_FAST_DIR>/act_infer}" # 放在快盘上的环境/检查点副本（可选，没有就用 conda 环境）
TRAIN_NAME="${TRAIN_NAME:-<TRAIN_NAME>}"        # 默认检查点所属的训练名
# ------------------------------------------------
CKPT="${1:-$ACT_DIR/outputs/train/$TRAIN_NAME/checkpoints/last/pretrained_model}"
[ $# -gt 0 ] && shift
if [ ! -f "$CKPT/config.json" ] || [ ! -f "$CKPT/model.safetensors" ]; then
  echo "找不到检查点：$CKPT（需要 config.json 和 model.safetensors）" >&2
  exit 1
fi
CKPT="$(readlink -f "$CKPT")"   # 解析 checkpoints/last 这类软链接，得到真实的步数目录

if [ -f "$INFER/env/.act_infer_ready" ] && [ -d "$INFER/lerobot_src/lerobot" ]; then
  PY="$INFER/env/bin/python"
  export PYTHONPATH="$INFER/lerobot_src${PYTHONPATH:+:$PYTHONPATH}"
  step_dir="$(dirname "$CKPT")"
  name="$(basename "$(dirname "$(dirname "$step_dir")")")_$(basename "$step_dir")"
  dest="$INFER/ckpt/$name"
  echo "同步检查点到 NVMe：$CKPT → $dest"
  mkdir -p "$dest"
  rsync -a --delete "$CKPT/" "$dest/"
  CKPT="$dest"
else
  echo "没找到 $INFER 下的环境副本，改用 conda 环境 ACT（在 /data0 上，启动可能很慢）" >&2
  source "$CONDA_DIR/etc/profile.d/conda.sh"
  conda activate ACT
  PY=python
fi

export CUDA_VISIBLE_DEVICES="${GPU:-0}"
cd "$here"
mkdir -p logs
log="logs/server_$(date +%Y%m%d_%H%M%S).log"
echo "Python $PY | 检查点 $CKPT | GPU $CUDA_VISIBLE_DEVICES | 127.0.0.1:${PORT:-18151} | 日志 $here/$log"
"$PY" act_policy_server.py \
  --host=127.0.0.1 --port="${PORT:-18151}" \
  --fps=30 --inference_latency=0.033 --obs_queue_timeout=2 \
  --policy_path="$CKPT" --device=cuda \
  "$@" 2>&1 | tee -i "$log"
