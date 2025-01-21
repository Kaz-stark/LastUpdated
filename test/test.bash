#!/bin/bash
#SPDX-FileCopyrightText: 2025 Kazuya Ochiai
#SPDX-License-Indentifier: BSD-3-Clause


dir=~
[ "$1" != "" ] && dir="$1"

# テスト用のダミーファイルとディレクトリを作成
mkdir -p /root/AppData/Local/LINE/bin
touch /root/AppData/Local/LINE/bin/LineLauncher.exe

# ビルドと環境設定
cd $dir/ros2_ws
colcon build
source $dir/.bashrc

# 正常パスでのテスト
ros2 run last_updated pub_node > test_publish.log 2>&1 & PUB_PID=$!
sleep 6
kill $PUB_PID
# ログの内容を表示(デバッグ用)
echo "=== Log Contents ==="
cat test_publish.log
echo "==================="

if grep -q "Monitoring LINE updates at:" test_publish.log; then
  echo "Published test passed"
else
  echo "failed publish test"
  echo "Log dose not contain expected message"
  exit 1
fi



# テスト実行前にログファイルが存在する場合は削除
rm -f test_publish.log

# 無効なパスでのテスト
export TEST_INVALID_PATH="/dummy/path/LINE"
ros2 run last_updated pub_node > test_invalid.log 2>&1 || true
if grep -q "LINE application not found at:" test_invalid.log; then
  echo "Invalid path test passed"
else
  echo "failed invalid path test"
  exit 1
fi

# トピックの購読テスト
timeout 10 ros2 run last_updated pub_node & NODE_PID=$!
sleep 6
timeout 10 ros2 topic echo /line_last_update > test_topic.log & LISTEN_PID=$!
sleep 6
kill -SIGKILL $NODE_PID || true
kill -SIGKILL $LISTEN_PID || true

if grep -q "data:" test_topic.log; then
  echo "Topic subscription test passed"
  exit 0
else
  echo "Topic subscription test failed"
  exit 1
fi

# クリーンアップ
rm -rf test_*.log
