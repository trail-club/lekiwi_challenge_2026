"""スキャンデータを角度範囲でフィルタリングし、短い欠損を補完する。

LeKiwi の車輪は 3 つ (左 +60°、後ろ 180°、右 −60°) なので、機体が映らない
のは 右後ろ・前方・左後ろ の 3 区間になる。この 3 区間だけを有効とし、それ以外の
角度の測距値を inf(障害物なし)に置き換えて /scan_filtered に転送する。
区間は lidar_angle で実測した値を入れる。

そのあと、欠損 (inf / nan) が max_interp_gap ビーム以下で、両隣に有効な値が
あるときは線形補完する。マスクした点は補完せず、隣としても使わないので、
機体の点や区間の外をまたいで補完することは無い。

Nav2 のコストマップ・collision_monitor・AMCL・slam_toolbox がこのトピックを
購読することで、車輪・ボディが誤って障害物としてマーキングされるのを防ぐ。

★ SLAM もこちらを使う (slam_toolbox.yaml の scan_topic)。地図に自機が
  焼き付くのを避けるため。地図とそれを突き合わせる AMCL (nav2.yaml の
  amcl.scan_topic) も同じトピックにしてある。

パラメータ:
    valid_ranges_deg (float[], 既定 [-170, -70, -50, 50, 70, 170]):
        有効区間 [deg] を [min1, max1, min2, max2, min3, max3] の平たい配列で
        指定する (右後ろ・前方・左後ろ)。両端を含む。min > max の区間は ±180°
        をまたぐとみなす。★ 既定値は車輪の方位から 10° ずつ余裕を取った仮の値。
        lidar_angle の出力で置き換えること。
    max_interp_gap (int, 既定 2): この本数以下の連続した欠損を補完する。0 で無効
"""

from __future__ import annotations

import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from lekiwi_base_bringup.scan_preprocess import (
    interpolate_short_gaps,
    mask_outside_intervals,
    parse_intervals_deg,
)


class ScanFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_angular_filter")

        # 右後ろ・前方・左後ろ [deg]
        self.declare_parameter("valid_ranges_deg", [-154.9, -140.9, -57.7, 93.8, 158.9, 171.0])
        self.declare_parameter("max_interp_gap", 2)

        flat = [float(v) for v in self.get_parameter("valid_ranges_deg").value]
        self._intervals = parse_intervals_deg(flat)
        self._max_gap = int(self.get_parameter("max_interp_gap").value)

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

        ranges_str = ", ".join(
            f"{math.degrees(lo):.1f}° 〜 {math.degrees(hi):.1f}°" for lo, hi in self._intervals
        )
        self.get_logger().info(
            f"スキャンフィルター起動: 有効範囲 [{ranges_str}], "
            f"補完 {self._max_gap} ビーム以下"
        )

    def _cb(self, msg: LaserScan) -> None:
        out = copy.copy(msg)
        out.ranges = list(msg.ranges)
        if msg.intensities:
            out.intensities = list(msg.intensities)

        inside = mask_outside_intervals(
            out.ranges, msg.angle_min, msg.angle_increment, self._intervals
        )
        interpolate_short_gaps(out.ranges, self._max_gap, msg.range_min, msg.range_max, inside)

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
