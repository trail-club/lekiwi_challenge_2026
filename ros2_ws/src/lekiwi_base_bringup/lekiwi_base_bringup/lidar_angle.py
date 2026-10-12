"""
ライダーデータをフィルタする際の角度を実測する。

右車輪、左車輪、後ろ車輪など機体が映ってしまうとこのライダーの計測値はinfに書き換えたほうがいい。
本スクリプトでは、機体が映らない角度を実測する。具体的には白い壁でlekiwiを囲った状態で
100回計測して90回以上計測不能/距離が0.2メートル以下だった点は機体が映っているとみなす。

/scan を num_scans 回受け取ったら、機体が映らない区間のうち長い順に 3 つを
右後ろ・前方・左後ろとしてログに出し、終了する。角度は正面が 0、左回りが正。

    ros2 run lekiwi_base_bringup lidar_angle

パラメータ:
    angle_min (float): ライダーの計測範囲の下限 [rad]
    angle_max (float): ライダーの計測範囲の上限 [rad]
    angle_increment (float): ライダーの計測の細かさ [rad]
    num_scans (int, 既定 100): 計測回数
    blocked_ratio (float, 既定 0.9): この割合以上 計測不能/近距離 なら機体とみなす
    blocked_distance (float, 既定 0.2): これ以下の距離は機体とみなす [m]

★ 角度は受信メッセージの angle_min / angle_increment ではなくパラメータから計算する。
  既定値は lidar_example.yaml (実機の /scan) の値。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# 中心角の小さい順 (右回り側から) に並べたときの名前
_LABELS = ("右後ろ", "前方", "左後ろ")


def is_blocked(r: float, blocked_distance: float) -> bool:
    """計測不能 (inf / nan) か、近すぎる点なら True。"""
    return not math.isfinite(r) or r <= blocked_distance


def find_visible_intervals(blocked: list[bool], circular: bool) -> list[tuple[int, int]]:
    """機体が映らない点の連続区間を (開始 index, 終了 index) で返す。

    circular なら末尾と先頭の区間をつなげる。その場合の終了 index は
    len(blocked) 以上になりうる (角度を求めるときに剰余を取ること)。
    """
    n = len(blocked)
    intervals: list[tuple[int, int]] = []
    start = None
    for i, b in enumerate(blocked):
        if not b and start is None:
            start = i
        elif b and start is not None:
            intervals.append((start, i - 1))
            start = None
    if start is not None:
        intervals.append((start, n - 1))

    # 全点が見えている場合はつなげない (一周まるごとの区間が 1 つ)
    if circular and len(intervals) >= 2 and intervals[0][0] == 0 and intervals[-1][1] == n - 1:
        head = intervals.pop(0)
        last_start, _ = intervals.pop()
        intervals.append((last_start, head[1] + n))
    return intervals


class LidarAngle(Node):
    def __init__(self) -> None:
        super().__init__("lidar_angle")

        self.declare_parameter("angle_min", -3.1415927410125732)
        self.declare_parameter("angle_max", 3.1415927410125732)
        self.declare_parameter("angle_increment", 0.017501909285783768)
        self.declare_parameter("num_scans", 100)
        self.declare_parameter("blocked_ratio", 0.9)
        self.declare_parameter("blocked_distance", 0.2)

        self._angle_min = float(self.get_parameter("angle_min").value)
        angle_max = float(self.get_parameter("angle_max").value)
        self._inc = float(self.get_parameter("angle_increment").value)
        self._num_scans = int(self.get_parameter("num_scans").value)
        self._blocked_count = int(self._num_scans * float(self.get_parameter("blocked_ratio").value))
        self._blocked_distance = float(self.get_parameter("blocked_distance").value)

        # 両端を含むので +1。lidar_example.yaml では 2π / 0.0175 + 1 = 360 点
        self._n_points = round((angle_max - self._angle_min) / self._inc) + 1
        # 一周ぶん (2π) 計測しているなら、末尾と先頭の区間は同じ方向でつながっている
        self._circular = self._n_points * self._inc >= 2.0 * math.pi - self._inc / 2.0

        self._counts = [0] * self._n_points
        self._received = 0
        self.done = False

        # scan_filter.py と同じく BEST_EFFORT で購読する (RELIABLE だと
        # BEST_EFFORT の publisher から受け取れない)
        self._sub = self.create_subscription(
            LaserScan, "scan", self._cb, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"計測開始: {self._num_scans} 回 / {self._n_points} 点, "
            f"{self._blocked_count} 回以上 計測不能 or {self._blocked_distance} m 以下 → 機体"
        )

    def _cb(self, msg: LaserScan) -> None:
        if self.done:
            return
        if len(msg.ranges) != self._n_points:
            # パラメータとメッセージが食い違っていると角度がずれるので数えない
            self.get_logger().warn(
                f"点数がパラメータと違うので捨てる: 受信 {len(msg.ranges)} 点 / "
                f"想定 {self._n_points} 点"
            )
            return

        for i, r in enumerate(msg.ranges):
            if is_blocked(r, self._blocked_distance):
                self._counts[i] += 1
        self._received += 1

        if self._received % 10 == 0:
            self.get_logger().info(f"{self._received}/{self._num_scans}")
        if self._received >= self._num_scans:
            self._report()
            self.done = True

    def _angle(self, index: int) -> float:
        return self._angle_min + (index % self._n_points) * self._inc

    def _report(self) -> None:
        blocked = [c >= self._blocked_count for c in self._counts]
        intervals = find_visible_intervals(blocked, self._circular)

        if not intervals:
            self.get_logger().error("機体が映らない点が 1 つも無い")
            return
        if len(intervals) != 3:
            self.get_logger().warn(
                f"見える区間が {len(intervals)} 個ある (想定は 3 個)。長い順に 3 個だけ出す"
            )

        # 長い順に 3 つ選び、中心角の順に並べて右後ろ・前方・左後ろと名付ける
        longest = sorted(intervals, key=lambda iv: iv[1] - iv[0], reverse=True)[:3]

        def center(iv: tuple[int, int]) -> float:
            c = self._angle_min + (iv[0] + iv[1]) / 2.0 * self._inc
            # 末尾と先頭をつなげた区間は π を超えうるので -π 〜 π に正規化
            return (c + math.pi) % (2.0 * math.pi) - math.pi

        longest.sort(key=center)
        labels = _LABELS if len(longest) == 3 else [f"区間{k}" for k in range(len(longest))]

        lines = ["機体が映らない区間:"]
        for label, (s, e) in zip(labels, longest):
            a_min = self._angle(s)
            a_max = self._angle(e)
            lines.append(
                f"  {label}: min={a_min:.4f} rad ({math.degrees(a_min):.1f}°), "
                f"max={a_max:.4f} rad ({math.degrees(a_max):.1f}°), {e - s + 1} 点"
            )
        self.get_logger().info("\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarAngle()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # scan_filter.py と同じく Ctrl+C は正常終了扱い
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
