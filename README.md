my_robot
概要

PC上でPythonプログラムを実行し、SSH接続を利用してRaspberry Piにロボットの移動命令を送信する制御システムです。

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
ファイル構成
text
lightrover_controller/
├── main.py
└── README.md

実行方法
bash
python main.py

システム構成

PCからSSH接続を利用してRaspberry Piに命令を送信し、Raspberry Pi側のROS環境でロボットを制御します。
