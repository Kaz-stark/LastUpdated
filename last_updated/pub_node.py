#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Kazuya Ochiai
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FileMonitorNode(Node):
    def __init__(self, target_path: str):
        super().__init__('file_monitor_node')
        
        # パブリッシャーの設定
        self.publisher = self.create_publisher(
            String,
            'file_last_modified',
            10
        )
        
        # 監視対象のパスを設定
        self.target_path = Path(target_path)
        if not self.target_path.exists():
            self.get_logger().error(f'Path does not exist: {target_path}')
            return
            
        # タイマーの設定（1秒間隔でチェック）
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.last_modified = None
        
        self.get_logger().info(f'Monitoring file: {self.target_path}')
        
    def timer_callback(self):
        try:
            # 最終更新日時を取得
            current_modified = datetime.fromtimestamp(
                os.path.getmtime(self.target_path)
            )
            
            # 前回と異なる場合のみパブリッシュ
            if self.last_modified != current_modified:
                msg = String()
                msg.data = current_modified.isoformat()
                self.publisher.publish(msg)
                self.last_modified = current_modified
                
                self.get_logger().info(f'Published last modified: {msg.data}')
                
        except Exception as e:
            self.get_logger().error(f'Error checking file: {str(e)}')

def main(args=None):
    # ROS2の初期化
    rclpy.init(args=args)
    
    # コマンドライン引数からファイルパスを取得
    if len(sys.argv) < 2:
        print("Usage: ros2 run last_updated pub_node <path_to_file>")
        return
    
    file_path = sys.argv[1]
    
    # ノードの作成と実行
    node = FileMonitorNode(file_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
