ishii@DESKTOP-79V73LO:~$ cat right_hand_05.py
# -*- coding: utf-8 -*-
"""
単純な右手法（右壁追従）探索ノード + 狭い隙間の壁認識
ROS 1 Melodic / Python 2.7 対応

入力:
- Cartographer: /scan_matched_points2 (PointCloud2)
- /odom: ロボット位置・向き

速度指令:
- SSH経由でラズパイの receive_cmd.py に送信

操作:
- Space: 自律移動 ON/OFF
- q: 終了
"""

import math
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import paramiko
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion


# ==========================================
# 状態定義
# ==========================================

FOLLOW = "右壁追従"
TURN_LEFT = "左旋回"
ADVANCE = "壁なし後の直進"
TURN_RIGHT = "右旋回"
SEARCH = "右壁探索"


class RightHandRuleExplorer:

    def __init__(self):

        rospy.init_node('right_hand_rule_explorer', anonymous=True)

        # ==========================================
        # SSH設定
        # ==========================================

        self.ssh_host = rospy.get_param(
            '~ssh_host', '192.168.10.20')

        self.ssh_user = rospy.get_param(
            '~ssh_user', 'ishii')

        self.ssh_password = os.environ.get(
            'LIGHTROVER_SSH_PASSWORD',
            rospy.get_param('~ssh_password', 'Shin1128')
        )

        self.remote_cmd = rospy.get_param(
            '~remote_cmd',
            "bash -c 'export ROS_MASTER_URI=http://192.168.10.20:11311 && "
            "export ROS_HOSTNAME=192.168.10.20 && "
            "source ~/catkin_ws/devel/setup.bash && "
            "python ~/catkin_ws/src/lightrover_ros/scripts/receive_cmd.py'"
        )

        # ==========================================
        # センサ設定
        # ==========================================

        self.cloud_topic = rospy.get_param(
            '~cloud_topic', '/scan_matched_points2')

        self.min_range = rospy.get_param(
            '~min_range', 0.05)

        self.max_range = rospy.get_param(
            '~max_range', 4.0)

        self.pose_source = rospy.get_param(
            '~pose_source', 'odom')

        self.base_frame = rospy.get_param(
            '~base_frame', 'base_link')

        self.debug = rospy.get_param(
            '~debug', True)

        # ==========================================
        # 距離パラメータ [m]
        # ==========================================

        # 右壁の検出距離
        self.wall_threshold = rospy.get_param(
            '~wall_threshold', 0.25)

        # 前方障害物の判定距離
        self.front_threshold = rospy.get_param(
            '~front_threshold', 0.20)

        # 前方判定の幅
        self.front_half_width = rospy.get_param(
            '~front_half_width', 0.08)

        # 右側の点群を取得する範囲
        self.right_x_half = rospy.get_param(
            '~right_x_half', 0.10)

        # 右壁がなくなったと判断する時間
        self.wall_lost_hold = rospy.get_param(
            '~wall_lost_hold', 2.0)

        # 右壁がなくなった後の直進距離
        self.advance_dist = rospy.get_param(
            '~advance_dist', 0.16)

        # ==========================================
        # 隙間(幅の狭い通路)を壁とみなす設定
        # ==========================================

        # この幅以下の隙間は通れない壁として扱う
        self.gap_width_max = rospy.get_param(
            '~gap_width_max', 0.25)

        # 隙間を探す前方の距離範囲 [m]
        # (xがこの範囲に入ったら、手前で左旋回する)
        self.gap_x_min = rospy.get_param(
            '~gap_x_min', 0.10)

        self.gap_trigger_dist = rospy.get_param(
            '~gap_trigger_dist', 0.25)

        # 隙間の左右の縁として探す横方向の範囲
        self.gap_search_half = rospy.get_param(
            '~gap_search_half', 0.30)

        # 隙間とみなす最小幅（これより狭い点の隙間は点群の粗さとして無視）
        self.gap_width_min = rospy.get_param(
            '~gap_width_min', 0.12)

        # 隙間の縁の2点が同じ壁面上にあるとみなすx方向の差
        self.gap_plane_tol = rospy.get_param(
            '~gap_plane_tol', 0.06)

        # 隙間の中心がロボット正面からずれてよい範囲
        self.gap_center_tol = rospy.get_param(
            '~gap_center_tol', 0.10)

        # 縁の外側に壁が続いているかを見る範囲(縁からの横距離の最小・最大)
        self.gap_edge_margin = rospy.get_param(
            '~gap_edge_margin', 0.03)

        self.gap_edge_extent = rospy.get_param(
            '~gap_edge_extent', 0.15)

        # 縁の外側に必要な点の数（片側ずつ）
        self.gap_edge_min_pts = rospy.get_param(
            '~gap_edge_min_pts', 3)

        # 連続して何回検出したら壁扱いにするか
        self.gap_confirm = rospy.get_param(
            '~gap_confirm', 3)

        # ==========================================
        # 右壁と平行に進む設定
        # ==========================================

        # 壁の向きを求めるために使う右側の点の前後範囲 [m]
        # (点群が約10cm間隔と粗いため、±0.25 = 前後50cmで見る)
        self.align_x_half = rospy.get_param(
            '~align_x_half', 0.10)

        # 直線を当てはめるのに必要な点の数
        self.align_min_pts = rospy.get_param(
            '~align_min_pts', 3)

        # 点のx方向の広がりの最小値 [m]（これ未満は向きを求めない）
        self.align_min_span = rospy.get_param(
            '~align_min_span', 0.10)

        # 直線からのばらつきの許容値 [m]（超えたら壁とみなさない）
        self.align_max_resid = rospy.get_param(
            '~align_max_resid', 0.02)

        # 角度のずれ1radあたりの旋回速度 [rad/s]
        self.k_align = rospy.get_param(
            '~k_align', 1.0)

        # 平行合わせの旋回速度の上限 [rad/s]
        self.w_align_max = rospy.get_param(
            '~w_align_max', 0.15)

        # この角度[deg]以下のずれは補正しない
        self.align_deadband_deg = rospy.get_param(
            '~align_deadband_deg', 2.5)

        # この角度[deg]以上ずれていたら、止まってその場で向きを直す
        # (止まったあとは、align_deadband_deg 以下になるまで直し続ける)
        self.align_turn_in_place_deg = rospy.get_param(
            '~align_turn_in_place_deg', 5.0)

        # 右壁からこの距離[m]以上離れないようにする
        # (超えたら、壁に近づく向きに少し傾けて進む)
        self.wall_dist_max = rospy.get_param(
            '~wall_dist_max', 0.20)

        # 壁に近づくために傾ける角度の上限 [deg]
        self.approach_max_deg = rospy.get_param(
            '~approach_max_deg', 8.0)

        # 上限の角度まで傾ける、wall_dist_max からの超過距離 [m]
        self.approach_full_excess = rospy.get_param(
            '~approach_full_excess', 0.05)

        # ==========================================
        # 速度パラメータ
        # ==========================================

        self.v_follow = rospy.get_param(
            '~v_follow', 0.01)

        # 右旋回角度
        self.right_turn_deg = rospy.get_param(
            '~right_turn_deg', 90.0)

        # 左旋回角度（1回の旋回で回る角度。まだ塞がっていれば続けてもう1回）
        self.left_turn_deg = rospy.get_param(
            '~left_turn_deg', 90.0)

        # 旋回速度
        self.w_turn = rospy.get_param(
            '~w_turn', 0.3)

        # ==========================================
        # 安全・終了設定
        # ==========================================

        self.scan_timeout = rospy.get_param(
            '~scan_timeout', 0.5)

        self.max_time = rospy.get_param(
            '~max_time', 0.0)

        # ==========================================
        # センサ値
        # ==========================================

        self.front_dist = float('inf')
        self.right_dist = float('inf')

        # 前方の狭い隙間
        self.gap_ahead = False
        self.gap_width = float('inf')
        self.gap_count = 0

        # 右壁に対するロボットの向きのずれ [rad]
        # (+ = 壁に向かって進んでいる / None = 求められない)
        self.wall_angle = None

        # 止まって向きを直している最中か
        self.aligning = False

        self.last_scan_time = None
        self.right_pts = ""
        self.no_points = False

        # ==========================================
        # ロボット位置
        # ==========================================

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.have_odom = False

        self.tf_listener = (
            tf.TransformListener()
            if self.pose_source == 'tf' else None
        )

        # ==========================================
        # 状態管理
        # ==========================================

        self.state = FOLLOW
        self.is_active = False

        # ==========================================
        # 評価ログ
        # ==========================================

        self.active_time = 0.0
        self.last_loop_time = None

        self.total_distance = 0.0
        self.total_turn = 0.0

        self.prev_odom = None

        self.turn_count = 0
        self.right_turn_count = 0

        # ==========================================
        # 壁がなくなったときの動作管理
        # ==========================================

        self.lost_since = None

        self.adv_start = (0.0, 0.0)
        self.adv_start_time = 0.0

        # ==========================================
        # 旋回管理
        # ==========================================

        self.turn_acc = 0.0
        self.turn_last_yaw = 0.0
        self.turn_start_time = 0.0

        # ==========================================
        # SSH
        # ==========================================

        self.ssh = None
        self.ssh_stdin = None
        self.settings = None

        # ==========================================
        # ROS Subscriber
        # ==========================================

        rospy.Subscriber(
            self.cloud_topic,
            PointCloud2,
            self.pc_callback,
            queue_size=1
        )

        rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_callback,
            queue_size=1
        )

    # ==========================================
    # SSH接続
    # ==========================================

    def connect_ssh(self):

        self.ssh = paramiko.SSHClient()

        self.ssh.set_missing_host_key_policy(
            paramiko.AutoAddPolicy()
        )

        try:

            self.ssh.connect(
                hostname=self.ssh_host,
                username=self.ssh_user,
                password=self.ssh_password
            )

            self.ssh_stdin, ssh_stdout, ssh_stderr = (
                self.ssh.exec_command(self.remote_cmd)
            )

            rospy.sleep(1.0)

            if ssh_stdout.channel.exit_status_ready():

                rospy.logerr(
                    "ラズパイ側のスクリプトが起動直後に終了しました:\n%s",
                    ssh_stderr.read()
                )

                return False

            rospy.loginfo(
                "右手法ノードが起動しました。"
            )

            return True

        except Exception as e:

            rospy.logerr(
                "SSH接続に失敗しました: %s",
                str(e)
            )

            return False

    # ==========================================
    # 速度指令送信
    # ==========================================

    def send_cmd(self, cmd):

        msg = "{:.3f},{:.3f}".format(
            cmd.linear.x,
            cmd.angular.z
        )

        try:

            self.ssh_stdin.write(msg + "\n")
            self.ssh_stdin.flush()

        except Exception as e:

            rospy.logerr(
                "SSH送信エラー: %s",
                str(e)
            )

    def stop_robot(self):

        for _ in range(5):

            self.send_cmd(Twist())
            time.sleep(0.05)

    # ==========================================
    # キーボード操作
    # ==========================================

    def check_keyboard(self):

        if select.select([sys.stdin], [], [], 0)[0]:

            tty.setraw(sys.stdin.fileno())

            key = sys.stdin.read(1)

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.settings
            )

            return key

        return None

    # ==========================================
    # ロボット位置取得
    # ==========================================

    def get_robot_pose(self, frame_id):

        if self.pose_source == 'tf':

            try:

                trans, rot = self.tf_listener.lookupTransform(
                    frame_id,
                    self.base_frame,
                    rospy.Time(0)
                )

                return (
                    trans[0],
                    trans[1],
                    euler_from_quaternion(rot)[2]
                )

            except tf.Exception:

                return None

        if not self.have_odom:

            return None

        return (
            self.robot_x,
            self.robot_y,
            self.robot_yaw
        )

    # ==========================================
    # 最小距離計算
    # ==========================================

    @staticmethod
    def window_min(values, mask):

        if not np.any(mask):

            return float('inf')

        return float(np.min(values[mask]))

    # ==========================================
    # 点群処理
    # ==========================================

    def find_wall_angle(self, xr, yr):
        """
        右側の点(前後 ±align_x_half)に直線 y = m*x + c を当てはめ、
        壁に対するロボットの向きのずれ [rad] を返す。求められなければ None。

        戻り値が + のとき、ロボットは壁に向かって進んでいる(左へ直す必要がある)。
        """

        mask = (
            (yr < 0.0) &
            (yr >= -self.wall_threshold) &
            (np.abs(xr) <= self.align_x_half)
        )

        if np.count_nonzero(mask) < self.align_min_pts:
            return None

        px = xr[mask]
        py = yr[mask]

        if (np.max(px) - np.min(px)) < self.align_min_span:
            return None

        m, c = np.polyfit(px, py, 1)

        resid = py - (m * px + c)

        if float(np.sqrt(np.mean(resid ** 2))) > self.align_max_resid:
            return None

        return math.atan(m)

    def find_gap(self, xr, yr):
        """
        前方の壁面にあいた「狭い穴」を探す。見つかれば幅[m]、なければ None。

        隙間と認める条件（すべて満たす場合のみ）:
        1. y方向に並べた隣り合う2点の間が gap_width_min~gap_width_max
           (点がまばらな穴であること。連続した壁なら間隔は小さい)
        2. 穴の中心がロボット正面付近
        3. 穴の両縁の2点が同じ壁面上(xがほぼ同じ)
        4. 両縁の外側にも、同じ壁面の点が続いている
           (通路の側壁のように外側に壁がない場合は除外)
        """

        band = (
            (xr >= self.gap_x_min) &
            (xr <= self.gap_trigger_dist) &
            (np.abs(yr) <= self.gap_search_half + self.gap_edge_extent)
        )

        bx = xr[band]
        by = yr[band]

        if len(by) < 2 * self.gap_edge_min_pts + 2:
            return None

        order = np.argsort(by)
        bx = bx[order]
        by = by[order]

        dys = np.diff(by)

        cand = np.where(
            (dys >= self.gap_width_min) &
            (dys <= self.gap_width_max)
        )[0]

        for i in cand:

            # 穴の中心が正面付近か
            center = 0.5 * (by[i] + by[i + 1])

            if abs(center) > self.gap_center_tol:
                continue

            # 両縁が同じ壁面上か
            if abs(bx[i] - bx[i + 1]) > self.gap_plane_tol:
                continue

            plane_x = 0.5 * (bx[i] + bx[i + 1])

            same_plane = (
                np.abs(bx - plane_x) <= self.gap_plane_tol
            )

            # 縁の外側に同じ壁面の点が続いているか
            lower = (
                same_plane[:i + 1] &
                (by[:i + 1] <= by[i] - self.gap_edge_margin) &
                (by[:i + 1] >= by[i] - self.gap_edge_extent)
            )

            upper = (
                same_plane[i + 1:] &
                (by[i + 1:] >= by[i + 1] + self.gap_edge_margin) &
                (by[i + 1:] <= by[i + 1] + self.gap_edge_extent)
            )

            if (
                np.count_nonzero(lower) >= self.gap_edge_min_pts and
                np.count_nonzero(upper) >= self.gap_edge_min_pts
            ):

                return float(dys[i])

        return None

    def pc_callback(self, msg):

        pose = self.get_robot_pose(
            msg.header.frame_id
        )

        if pose is None:
            return

        x0, y0, yaw = pose

        pts = list(
            pc2.read_points(
                msg,
                field_names=("x", "y"),
                skip_nans=True
            )
        )

        if len(pts) == 0:

            self.no_points = True

            self.front_dist = float('inf')
            self.right_dist = float('inf')

            self.gap_ahead = False
            self.gap_width = float('inf')
            self.gap_count = 0
            self.wall_angle = None

            self.last_scan_time = rospy.Time.now()

            return

        arr = np.array(pts)

        # マップ座標からロボット座標へ変換

        dx = arr[:, 0] - x0
        dy = arr[:, 1] - y0

        c = math.cos(yaw)
        s = math.sin(yaw)

        xr = c * dx + s * dy
        yr = -s * dx + c * dy

        dist = np.hypot(xr, yr)

        # 有効距離の点群のみ使用

        valid = (
            (dist >= self.min_range) &
            (dist <= self.max_range)
        )

        xr = xr[valid]
        yr = yr[valid]

        self.no_points = (len(xr) == 0)

        if self.no_points:

            self.front_dist = float('inf')
            self.right_dist = float('inf')

            self.gap_ahead = False
            self.gap_width = float('inf')
            self.gap_count = 0
            self.wall_angle = None

            self.last_scan_time = rospy.Time.now()

            return

        # ==========================================
        # 前方の壁までの距離
        # ==========================================

        front_mask = (
            (xr > 0.0) &
            (np.abs(yr) <= self.front_half_width)
        )

        self.front_dist = self.window_min(
            xr,
            front_mask
        )

        # ==========================================
        # 右側の壁までの距離
        # ==========================================

        right_mask = (
            (yr < 0.0) &
            (np.abs(xr) <= self.right_x_half)
        )

        self.right_dist = self.window_min(
            -yr,
            right_mask
        )

        # ==========================================
        # 前方の隙間検出（幅の狭い隙間は壁扱い）
        # ==========================================
        # 前方の壁面にあいた穴(幅 gap_width_min~gap_width_max)を探す。
        # 誤検出を避けるため、連続 gap_confirm 回検出したら壁扱いにする

        raw_width = self.find_gap(xr, yr)

        if raw_width is not None:

            self.gap_count += 1
            self.gap_width = raw_width

        else:

            self.gap_count = 0
            self.gap_width = float('inf')

        self.gap_ahead = (self.gap_count >= self.gap_confirm)

        # ==========================================
        # 右壁に対する向きのずれ（平行に進むため）
        # ==========================================

        self.wall_angle = self.find_wall_angle(xr, yr)

        # デバッグ用点群

        idx = np.where(right_mask)[0]

        idx = idx[
            np.argsort(-yr[idx])
        ][:6]

        self.right_pts = " ".join(
            "(x%+.2f,横%.2f)" % (xr[i], -yr[i])
            for i in idx
        )

        self.last_scan_time = rospy.Time.now()

        if self.debug:

            rospy.loginfo_throttle(
                2.0,
                "[DEBUG] 前方=%.2fm 右側=%.2fm 隙間幅=%.2fm",
                self.front_dist,
                self.right_dist,
                self.gap_width
            )

    # ==========================================
    # オドメトリ
    # ==========================================

    def odom_callback(self, msg):

        nx = msg.pose.pose.position.x
        ny = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        nyaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )[2]

        if self.is_active and self.prev_odom is not None:

            self.total_distance += math.hypot(
                nx - self.prev_odom[0],
                ny - self.prev_odom[1]
            )

            dyaw = nyaw - self.prev_odom[2]

            self.total_turn += abs(
                math.atan2(
                    math.sin(dyaw),
                    math.cos(dyaw)
                )
            )

        self.prev_odom = (nx, ny, nyaw)

        self.robot_x = nx
        self.robot_y = ny
        self.robot_yaw = nyaw

        self.have_odom = True

    # ==========================================
    # 直進開始
    # ==========================================

    def start_advance(self):

        self.lost_since = None

        self.state = ADVANCE

        self.adv_start = (
            self.robot_x,
            self.robot_y
        )

        self.adv_start_time = rospy.get_time()

        rospy.loginfo(
            "右壁なし: %.2fm直進開始",
            self.advance_dist
        )

    # ==========================================
    # 右旋回開始
    # ==========================================

    def start_right_turn(self):

        self.state = TURN_RIGHT

        self.turn_acc = 0.0

        self.turn_last_yaw = self.robot_yaw

        self.turn_start_time = rospy.get_time()

        self.right_turn_count += 1

        rospy.loginfo(
            "右旋回開始: %.1f度",
            self.right_turn_deg
        )

    # ==========================================
    # 左旋回開始（left_turn_deg ずつ回る）
    # ==========================================

    def start_left_turn(self, reason):

        self.state = TURN_LEFT

        self.turn_acc = 0.0

        self.turn_last_yaw = self.robot_yaw

        self.turn_start_time = rospy.get_time()

        self.turn_count += 1

        self.lost_since = None

        rospy.loginfo(
            "%s: 左旋回開始 %.1f度",
            reason,
            self.left_turn_deg
        )

    # ==========================================
    # 状態更新
    # ==========================================

    def update_state(self):

        front = self.front_dist
        now = rospy.get_time()

        right_wall = (
            self.right_dist <= self.wall_threshold
        )

        # ==========================================
        # 左旋回中
        # ==========================================
        # left_turn_deg(90度)回ったら一旦旋回を終える。
        # まだ前方が塞がっている(または狭い隙間が正面にある)なら、
        # 続けてもう90度回る

        if self.state == TURN_LEFT:

            dyaw = (
                self.robot_yaw - self.turn_last_yaw
            )

            self.turn_acc += abs(
                math.atan2(
                    math.sin(dyaw),
                    math.cos(dyaw)
                )
            )

            self.turn_last_yaw = self.robot_yaw

            target = math.radians(
                self.left_turn_deg
            )

            timeout = (
                3.0 * target / self.w_turn
            )

            if (
                self.turn_acc >= target or
                now - self.turn_start_time > timeout
            ):

                blocked = (
                    front <= self.front_threshold + 0.10
                    or self.gap_ahead
                )

                if blocked:

                    self.start_left_turn("旋回後もまだ塞がっている")

                    return

                self.state = FOLLOW
                self.lost_since = None

                rospy.loginfo(
                    "左旋回終了: 壁追従再開"
                )

            else:

                return

        # ==========================================
        # 右旋回中
        # ==========================================

        elif self.state == TURN_RIGHT:

            dyaw = (
                self.robot_yaw - self.turn_last_yaw
            )

            self.turn_acc += abs(
                math.atan2(
                    math.sin(dyaw),
                    math.cos(dyaw)
                )
            )

            self.turn_last_yaw = self.robot_yaw

            target = math.radians(
                self.right_turn_deg
            )

            timeout = (
                3.0 * target / self.w_turn
            )

            if (
                self.turn_acc >= target or
                now - self.turn_start_time > timeout
            ):

                self.state = SEARCH

                rospy.loginfo(
                    "右旋回終了: 右壁探索開始"
                )

            else:

                return

        # 壁なし後の直進中に前方が詰まったら、左ではなく右旋回を優先
        if self.state == ADVANCE and front < self.front_threshold:

            rospy.loginfo("直進中に前方が詰まったため右旋回へ")

            self.start_right_turn()

            return

        # ==========================================
        # 前方障害物 / 狭い隙間の判定
        # ==========================================

        if front < self.front_threshold or self.gap_ahead:

            if self.state != TURN_LEFT:

                if self.gap_ahead:

                    reason = (
                        "前方に狭い隙間(%.2fm)を検出" % self.gap_width
                    )

                else:

                    reason = "前方障害物"

                self.start_left_turn(reason)

            return

        # ==========================================
        # 右旋回後の壁探索
        # ==========================================

        if self.state == SEARCH:

            if right_wall:

                self.state = FOLLOW

                self.lost_since = None

                rospy.loginfo(
                    "右壁を検出: 壁追従再開"
                )

            return

        # ==========================================
        # 右壁がない場合の判定
        # ==========================================

        if self.state == FOLLOW:

            if not right_wall:

                if self.lost_since is None:

                    self.lost_since = now

                    rospy.loginfo(
                        "右壁が見えなくなりました"
                    )

                lost_time = (
                    now - self.lost_since
                )

                # 一定時間経過したら直進

                if lost_time >= self.wall_lost_hold:

                    self.start_advance()

            else:

                self.lost_since = None

        # ==========================================
        # 一定距離直進
        # ==========================================

        elif self.state == ADVANCE:

            moved = math.hypot(
                self.robot_x - self.adv_start[0],
                self.robot_y - self.adv_start[1]
            )

            timeout = (
                3.0 * self.advance_dist / self.v_follow
            )

            if (
                moved >= self.advance_dist or
                now - self.adv_start_time > timeout
            ):

                self.start_right_turn()

    # ==========================================
    # 速度指令生成
    # ==========================================

    def make_cmd(self):

        cmd = Twist()

        # 左旋回

        if self.state == TURN_LEFT:

            cmd.linear.x = 0.0
            cmd.angular.z = self.w_turn

        # 右旋回

        elif self.state == TURN_RIGHT:

            cmd.linear.x = 0.0
            cmd.angular.z = -self.w_turn

        # 直進（右壁追従中は壁と平行になるよう向きを補正）

        else:

            cmd.linear.x = self.v_follow
            cmd.angular.z = 0.0

            if self.state == FOLLOW:

                # 壁から wall_dist_max より離れているとき、
                # 壁に近づく向き(+)の目標角度を作る(離れるほど大きく、上限あり)
                target = 0.0

                if (
                    self.right_dist <= self.wall_threshold
                    and self.right_dist > self.wall_dist_max
                ):

                    excess = self.right_dist - self.wall_dist_max

                    target = math.radians(self.approach_max_deg) * min(
                        1.0,
                        excess / self.approach_full_excess
                    )

                ang = self.wall_angle

                # 壁の向きが求められなくても、離れすぎなら近づく方向へ直す
                # (平行(0度)とみなす)
                if ang is None and target > 0.0:

                    ang = 0.0

                if ang is not None:

                    # 目標角度とのずれ。+ なら壁に向きすぎ(左へ)、- なら足りない(右へ)
                    err = ang - target

                    deg = abs(math.degrees(err))

                    if deg > self.align_deadband_deg:

                        w = self.k_align * err

                        w = max(
                            -self.w_align_max,
                            min(self.w_align_max, w)
                        )

                        cmd.angular.z = w

                        # 一定以上ずれたら止まってその場で向きを直す。
                        # 止まったあとは deadband 以下になるまで直し続ける
                        if deg >= self.align_turn_in_place_deg:

                            self.aligning = True

                        if self.aligning:

                            cmd.linear.x = 0.0

                    else:

                        self.aligning = False

                else:

                    self.aligning = False

            else:

                self.aligning = False

        return cmd

    # ==========================================
    # 点群の受信確認
    # ==========================================

    def scan_is_fresh(self):

        if self.last_scan_time is None:

            return False

        return (
            rospy.Time.now() - self.last_scan_time
        ).to_sec() < self.scan_timeout

    # ==========================================
    # メイン処理
    # ==========================================

    def run(self):

        try:

            self.settings = termios.tcgetattr(
                sys.stdin
            )

        except Exception:

            rospy.logerr(
                "標準入力が端末ではないため、キーボード操作を使えません。"
            )

            return

        if not self.connect_ssh():

            return

        print("\n" + "=" * 45)
        print(" 【右手法 操作ガイド】")
        print("  [Spaceキー] : 自律移動 ON / OFF")
        print("  [ q キー ]  : プログラム終了")
        print("=" * 45 + "\n")

        rate = rospy.Rate(10)

        try:

            while not rospy.is_shutdown():

                key = self.check_keyboard()

                # ==================================
                # 自律移動 ON / OFF
                # ==================================

                if key == ' ':

                    self.is_active = not self.is_active

                    if self.is_active:

                        self.state = FOLLOW
                        self.lost_since = None

                        self.prev_odom = (
                            self.robot_x,
                            self.robot_y,
                            self.robot_yaw
                        )

                        rospy.loginfo(
                            "自律移動 ON"
                        )

                    else:

                        self.lost_since = None

                        rospy.loginfo(
                            "自律移動 OFF"
                        )

                elif key == 'q':

                    rospy.loginfo(
                        "終了します"
                    )

                    break

                # ==================================
                # 探索時間計測
                # ==================================

                now = rospy.get_time()

                if (
                    self.last_loop_time is not None
                    and self.is_active
                ):

                    self.active_time += (
                        now - self.last_loop_time
                    )

                self.last_loop_time = now

                if (
                    self.max_time > 0
                    and self.active_time >= self.max_time
                ):

                    rospy.loginfo(
                        "設定時間に到達しました"
                    )

                    break

                # ==================================
                # 自律移動 OFF
                # ==================================

                if not self.is_active:

                    self.send_cmd(Twist())

                # ==================================
                # センサ異常時は停止
                # ==================================

                elif (
                    not self.scan_is_fresh()
                    or self.no_points
                ):

                    self.send_cmd(Twist())

                    rospy.logwarn_throttle(
                        2.0,
                        "点群または位置情報が取得できないため停止中"
                    )

                # ==================================
                # 自律移動
                # ==================================

                else:

                    self.update_state()

                    self.send_cmd(
                        self.make_cmd()
                    )

                    lost = (
                        0.0
                        if self.lost_since is None
                        else now - self.lost_since
                    )

                    rospy.loginfo_throttle(
                        0.5,
                        "[%s] 前方 %.2f m / 右 %.2f m "
                        "| 右壁なし %.1f s | 隙間幅 %.2f m(%s) "
                        "| 壁角度 %s deg | 右側点群: %s",
                        self.state,
                        self.front_dist,
                        self.right_dist,
                        lost,
                        self.gap_width,
                        "壁扱い" if self.gap_ahead else "-",
                        (
                            "%.1f" % math.degrees(self.wall_angle)
                            if self.wall_angle is not None else "-"
                        ),
                        self.right_pts if self.right_pts else "なし"
                    )

                rate.sleep()

        except rospy.ROSInterruptException:

            pass

        except Exception as e:

            rospy.logerr(
                "メインループエラー: %s",
                str(e)
            )

        finally:

            if self.settings is not None:

                termios.tcsetattr(
                    sys.stdin,
                    termios.TCSADRAIN,
                    self.settings
                )

            self.stop_robot()

            self.log_summary()

            try:

                if self.ssh_stdin is not None:
                    self.ssh_stdin.close()

                if self.ssh is not None:
                    self.ssh.close()

                rospy.loginfo(
                    "SSH接続を終了しました"
                )

            except Exception:

                pass

    # ==========================================
    # 実験結果表示
    # ==========================================

    def log_summary(self):

        rospy.loginfo(
            "\n===== 右手法 終了時 =====\n"
            "探索時間(自律ON累積) : %.1f s\n"
            "総走行距離           : %.2f m\n"
            "総旋回角度           : %.1f deg (%.2f rad)\n"
            "左旋回回数           : %d 回\n"
            "右旋回回数           : %d 回\n"
            "=========================="
            % (
                self.active_time,
                self.total_distance,
                math.degrees(self.total_turn),
                self.total_turn,
                self.turn_count,
                self.right_turn_count
            )
        )


# ==========================================
# メイン
# ==========================================

if __name__ == '__main__':

    try:

        RightHandRuleExplorer().run()

    except rospy.ROSInterruptException:

        pass
ishii@DESKTOP-79V73LO:~$ python right_hand_05.py _advance_dist:=0.10 _target_right_dist:=0.15