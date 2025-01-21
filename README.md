# LastUpdated
[![test](https://github.com/Kaz-stark/LastUpdated/actions/workflows/test.yml/badge.svg)](https://github.com/Kaz-stark/LastUpdated/actions/workflows/test.yml)  [![GitHub License](https://img.shields.io/github/license/KobayashiYusei/numberGuesser)](LICENSE) <img src="https://img.shields.io/badge/ -Python-F9DC3E.svg?logo=python">

## 概要
- LastUpdatedは課題2のために作られたコマンドです。  
- pub_nodeを実行すると、LINEの最新更新日を表示します。  

## 対応環境  
- OS  
    Ubuntu 22.04.1 LTS  

- ROS2  
    ROS 2 Humble  

## 使い方  
```
$ ros2 run last_updated pub_node
```  
- LINEファイルが更新されるたびに、2つ目のターミナルで新しいメッセージが表示されます。更新がない場合は新しいメッセージは表示されません。  

## 実行例  
```
$ ros2 run last_updated pub_node
[INFO] [1737276482.376753392] [line_update_monitor_node]: Monitoring LINE updates at: /path/to/your/LINE
```  

## 出力例  
```  
$ ros2 topic echo /line_last_updated
data: LINE Update Detected!
Last Modified: 2024-01-21T10:30:45.123456
Location: /path/to/your/LINE
---
```


## ライセンス  
- このソフトウェアパッケージは、[3条項BSDライセンス](https://github.com/Kaz-stark/LastUpdated/blob/main/LICENSE)の下、再頒布および使用が許可されます。  
- © 2025 Kazuya Ochiai  


## 使用させていただいたコンテナ  
- [ryuichiueda/ubuntu22.04-ros2:latest](https://hub.docker.com/r/ryuichiueda/ubuntu22.04-ros2)-ryuichiueda

### 参考資料
- [著作権とライセンス|上田隆一](https://ryuichiueda.github.io/slides_marp/robosys2024/lesson5.html#1)  
- [ソフトウェアのテスト|上田隆一](https://ryuichiueda.github.io/slides_marp/robosys2024/lesson6.html)  
- [GitHubでのテスト|上田隆一](https://ryuichiueda.github.io/slides_marp/robosys2024/lesson7.html#1)
- [Robot Operating System(ROS 2)|上田隆一](https://ryuichiueda.github.io/slides_marp/robosys2024/lesson8.html#24)  
- [ROSシステムのテスト|上田隆一](https://ryuichiueda.github.io/slides_marp/robosys2024/lesson10.html#4)  
- [ROS2 publisher/subscriber node(+topic)の作成|@Dorebom(dorebom b)](https://qiita.com/Dorebom/items/47fb67e5e47a205f1395)  
- [Pythonで学ぶROS2ノード開発：パブリッシャーとサブスクライバーの基本ROS2で通信する|Murasan Lab](https://murasan-net.com/2024/09/23/ros2-publisher-subscriber-python/)  
  
