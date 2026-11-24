my_robot
概要

PC上でPythonプログラム（inght_hand.py)を実行し、SSH接続を利用してRaspberry Piにロボットの移動命令を送信する制御システムです。
launchファイルはロボット本体の移動制御用につくったもの既存のコードを必要な分だけ起動する。

ロボットから点群データを受け取り、それをもとに右手法をベースにした探索アルゴリズムを実行するRviz上で作成した地図を確認できる。

開発環境
OS：Windows / Python実行環境
プログラミング言語：Python
通信方式：SSH
接続先：Raspberry Pi
ロボット制御：ROS
主な機能
Raspberry PiとのSSH接続
ロボットへの移動命令の送信
PC側からのロボット操作



実行方法
ロボット側
roslaunch lightrover_ros roadmap_with_lidar.launch

PC側
python right_hand.py

システム構成

PCからSSH接続を利用してRaspberry Piに命令を送信し、Raspberry Pi側のROS環境でロボットを制御します。
