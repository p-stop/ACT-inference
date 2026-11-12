#!/usr/bin/env bash
# 在 pro5000 上启动 ACT 远程推理服务端（只监听 127.0.0.1，双臂机器经 SSH 隧道连进来）。
# 用法：
#   bash start_server.sh                       # 默认 ACT_fmix 的最新检查点
#   bash start_server.sh ACT_bi                # 某个模型的最新检查点（checkpoints/last）
#   bash start_server.sh ACT_bi/240000         # 某个模型的某一步
#   bash start_server.sh /完整路径/pretrained_model
#   bash start_server.sh --list                # 列出所有可用的检查点
#   其余 --参数=值 原样传给 act_policy_server.py（例如 --similarity_atol=0）
# 环境变量：GPU=卡号（默认自动挑当前最空闲的卡），PORT=端口（默认 18151，改了要同时改隧道钥匙的 permitopen）
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ACT=<PRO5000_ACT_DIR>
TRAIN=$ACT/outputs/train
PY=$ACT/.venv/bin/python

if [ "${1:-}" = "--list" ]; then
  for m in "$TRAIN"/*/; do
    n=$(basename "$m"); steps=$(ls "$m/checkpoints" 2>/dev/null | grep -v last | tr '\n' ' ')
    echo "$n: $steps(last -> $(basename "$(readlink -f "$m/checkpoints/last")"))"
  done
  exit 0
fi

ARG="${1:-ACT_fmix}"
[ $# -gt 0 ] && shift
case "$ARG" in
  /*) CKPT="$ARG" ;;
  */*) CKPT="$TRAIN/${ARG%%/*}/checkpoints/$(printf '%06d' "$((10#${ARG##*/}))")/pretrained_model" ;;
  *) CKPT="$TRAIN/$ARG/checkpoints/last/pretrained_model" ;;
esac
if [ ! -f "$CKPT/config.json" ] || [ ! -f "$CKPT/model.safetensors" ]; then
  echo "找不到检查点：$CKPT（需要 config.json 和 model.safetensors）。用 --list 看有哪些。" >&2
  exit 1
fi
CKPT="$(readlink -f "$CKPT")"   # 解析 checkpoints/last，日志里记下真实的步数

if [ -z "${GPU:-}" ]; then
  # 挑利用率最低、其次已用显存最少的卡
  GPU=$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits \
        | sort -t, -k2,2n -k3,3n | head -1 | cut -d, -f1 | tr -d ' ')
fi
export CUDA_VISIBLE_DEVICES="$GPU"
cd "$here"
mkdir -p logs
log="logs/server_$(date +%Y%m%d_%H%M%S).log"
echo "检查点 $CKPT | GPU $CUDA_VISIBLE_DEVICES | 127.0.0.1:${PORT:-18151} | 日志 $here/$log"
"$PY" act_policy_server.py \
  --host=127.0.0.1 --port="${PORT:-18151}" \
  --fps=30 --inference_latency=0.033 --obs_queue_timeout=2 \
  --policy_path="$CKPT" --device=cuda \
  "$@" 2>&1 | tee -i "$log"
