#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Kazuya Ochiai
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys
from datetime import datetime
from pathlib import Path
import platform

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LineUpdateMonitorNode(Node):
    def __init__(self):
        super().__init__('line_update_monitor_node')
        
        # パブリッシャーの設定
        self.publisher = self.create_publisher(
            String,
            'line_last_update',
            10
        )
        
        # LINEアプリケーションのパスを設定
        self.line_path = self._get_line_path()
        if not self.line_path.exists():
            self.get_logger().error(f'LINE application not found at: {self.line_path}')
            return
            
        # タイマーの設定（1秒間隔でチェック）
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.last_modified = None
        self.get_logger().info(f'Monitoring LINE updates at: {self.line_path}')
    
    def _get_line_path(self) -> Path:
        """OneDriveのパスからLINEアプリケーションのパスを返す"""
        system = platform.system()
        home = Path.home()
        
    if system == 'Windows':
        # Windowsの場合のパス
        return Path(home) / 'AppData' / 'Local' / 'LINE' / 'bin' / 'LineLauncher.exe'
    elif system == 'Linux':
        # Linuxのテスト環境用のパス
        return Path('/root/AppData/Local/LINE/bin/LineLauncher.exe')
    elif system == 'Darwin':  # macOS
        return Path('/Applications/LINE.app')
    
    # デフォルトパス
    return Path(home) / 'AppData' / 'Local' / 'LINE' / 'bin' / 'LineLauncher.exe'  

        # パスの存在確認とデバッグ情報の出力
        self.get_logger().info(f'Checking LINE at: {onedrive_path}')
        
        if onedrive_path.exists():
            return onedrive_path
        
        # パスが見つからない場合、親ディレクトリの内容を確認（デバッグ用）
        parent_dir = onedrive_path.parent
        if parent_dir.exists():
            self.get_logger().info(f'Contents of {parent_dir}:')
            for file in parent_dir.glob('*'):
                self.get_logger().info(f'Found: {file.name}')
        
        return onedrive_path
    
    def timer_callback(self):
        try:
            # 最終更新日時を取得
            current_modified = datetime.fromtimestamp(
                os.path.getmtime(self.line_path)
            )
            
            # 前回と異なる場合のみパブリッシュ
            if self.last_modified != current_modified:
                msg = String()
                msg.data = (
                    f'LINE Update Detected!\n'
                    f'Last Modified: {current_modified.isoformat()}\n'
                    f'Location: {self.line_path}'
                )
                self.publisher.publish(msg)
                self.last_modified = current_modified
                self.get_logger().info(f'Published LINE update: {current_modified.isoformat()}')
        
        except Exception as e:
            self.get_logger().error(f'Error checking LINE updates: {str(e)}')

def main(args=None):
    # ROS2の初期化
    rclpy.init(args=args)
    
    # ノードの作成と実行
    node = LineUpdateMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
