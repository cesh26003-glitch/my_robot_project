my_robot
概要

LightRover(Raspberry Pi搭載)を使い、LiDARとCartographerによる地図作成を行いながら、右手法ベースで迷路を自律探索するシステムです。

自作した部分

右手法の探索プログラム(right_hand_05.py):Cartographerの点群(/scan_matched_points2)と /odom から前方・右側の壁との位置関係を判断し、速度指令を生成します。速度指令は SSH(paramiko)経由でRaspberry Pi側に送ります。
launchファイル(roadmap_with_lidar.launch):既存の移動制御ノードとCartographerのうち、必要なものだけを起動するようにまとめたものです。

既存のものを利用した部分

ロボットの移動制御用コード
Cartographer(SLAMによる地図作成。生成された地図はRvizで確認)

開発環境: Windows(WSL/Ubuntu)/ Python 2.7 / ROS 1 Melodic / SSH(paramiko)
実行方法: ロボット側 roslaunch lightrover_ros drive_with_lidar.launch、PC側 python right_hand.py(Spaceキーで自律移動 ON/OFF)
