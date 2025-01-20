# LastUpdated

# 概要
- LastUpdatedは課題2のために作られたコマンドです。  
- pub_nodeを実行すると、LINEの最新更新日を表示します。  

# 対応環境  
- OS  
    Ubuntu 22.04.1 LTS  

- ROS2  
    ROS 2 Humble  

# 使い方  
```
$ ros2 run last_updated pub_node
```  

# 実行例  
```
$ ros2 run last_updated pub_node
```  

# 出力例  
```  
$ ros2 topic echo /line_last_updated
data: 
```

# 参考  



# 使用させていただいたコンテナ  
- [ryuichiueda/ubuntu22.04-ros2:latest](https://hub.docker.com/r/ryuichiueda/ubuntu22.04-ros2)-ryuichiueda
