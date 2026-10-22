#!/bin/zsh
# Finderでダブルクリックして起動。
cd "$(dirname "$0")" || exit 1
export PATH="/opt/homebrew/bin:$PATH"
if curl -fsS --max-time 1 http://127.0.0.1:8765/api/state >/dev/null 2>&1; then
  open http://127.0.0.1:8765
  exit 0
fi
if ! command -v uv >/dev/null; then
  echo 'uvが見つかりません。uvをインストールしてから実行してください。'
  read -r '?Enterで閉じる'
  exit 1
fi
uv run air-viewer &
air_viewer_pid=$!
trap 'kill -TERM "$air_viewer_pid" 2>/dev/null' EXIT INT TERM
for attempt in {1..100}; do
  if curl -fsS --max-time 1 http://127.0.0.1:8765/api/state >/dev/null 2>&1; then
    open http://127.0.0.1:8765
    break
  fi
  if ! kill -0 "$air_viewer_pid" 2>/dev/null; then
    echo '起動できませんでした。上のエラーを確認してください。'
    read -r '?Enterで閉じる'
    exit 1
  fi
  sleep 0.2
done
wait "$air_viewer_pid"
