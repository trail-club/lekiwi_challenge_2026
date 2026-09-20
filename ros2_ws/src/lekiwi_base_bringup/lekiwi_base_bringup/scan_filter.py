"""スキャンデータを角度範囲でフィルタリングする。

右車輪(−60°)から左車輪(+60°)の間、すなわちロボット前方 120° アークのみを
有効とし、それ以外の角度の測距値を inf(障害物なし)に置き換えて /scan_filtered
に転送する。

Nav2 のコストマップ・collision_monitor・AMCL・slam_toolbox がこのトピックを
購読することで、後方の車輪・ボディが誤って障害物としてマーキングされるのを防ぐ。

★ SLAM もこちらを使う (slam_toolbox.yaml の scan_topic)。地図に自機が
  焼き付くのを避けるため。地図とそれを突き合わせる AMCL (nav2.yaml の
  amcl.scan_topic) も同じトピックにしてある。

パラメータ:
    angle_min_deg (float, 既定 -60.0): 有効範囲の下限 [deg]
    angle_max_deg (float, 既定  60.0): 有効範囲の上限 [deg]
"""

from __future__ import annotations

import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_angular_filter")

        self.declare_parameter("angle_min_deg", -60.0)
        self.declare_parameter("angle_max_deg", 60.0)

        self._a_min = math.radians(self.get_parameter("angle_min_deg").value)
        self._a_max = math.radians(self.get_parameter("angle_max_deg").value)

        # ★ 購読は BEST_EFFORT (qos_profile_sensor_data) にすること。
        #   depth=10 の既定は RELIABLE で、BEST_EFFORT の publisher からは
        #   **1 メッセージも受け取れない**。fake_scan は sensor_data QoS
        #   (= BEST_EFFORT) で出すので、RELIABLE のままだと sim_nav で
        #   /scan_filtered が publisher 1 / データ 0 という状態になり、
        #   slam_toolbox が map -> odom を永遠に出さない。
        #   RELIABLE な publisher (sllidar_node) からは BEST_EFFORT でも
        #   受け取れるので、こちらにしておけば実機・模擬の両方で動く。
        #
        #   配信側は逆に RELIABLE のままにする。RELIABLE な publisher は
        #   BEST_EFFORT の購読者にも RELIABLE の購読者にも届くので、
        #   Nav2 の costmap がどちらで購読していても互換になる。
        self._sub = self.create_subscription(
            LaserScan, "scan", self._cb, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(LaserScan, "scan_filtered", 10)

        self.get_logger().info(
            f"スキャンフィルター起動: 有効範囲 {math.degrees(self._a_min):.1f}° 〜 {math.degrees(self._a_max):.1f}°"
        )

    def _cb(self, msg: LaserScan) -> None:
        out = copy.copy(msg)
        out.ranges = list(msg.ranges)
        if msg.intensities:
            out.intensities = list(msg.intensities)

        for i in range(len(out.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            # -π 〜 π に正規化
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if angle < self._a_min or angle > self._a_max:
                out.ranges[i] = float("inf")

        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ★ Ctrl+C は正常終了。捕まえないと停止のたびにトレースバックと
        #   "process has died [exit code -2]" が launch のログに出る。
        #   このノードはハードウェアを持たないので安全性には影響しないが、
        #   統合 launch では停止経路が Ctrl+C 一本なので、正常な停止でログが
        #   赤くなると**本物の異常を見落とす**。fake_scan と同じ扱いに揃える。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
