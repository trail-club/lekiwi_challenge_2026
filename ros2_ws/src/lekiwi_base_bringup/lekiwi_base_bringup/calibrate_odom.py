"""エンコーダ odom を LiDAR の位置合わせと比べ、車輪半径・ベース半径を較正する。

nav.launch.py (実機) が動いている状態で呼ぶ。★ ロボットが実際に動く。

    ros2 run lekiwi_base_bringup calibrate_odom
    ros2 run lekiwi_base_bringup calibrate_odom --ros-args -p linear_distance:=0.2

既定の動作 (各動作の前後で静止してスキャンを取る):
  前進 → 後退 → 左 → 右 を ``linear_repeats`` 回、
  左 360° → 右 360° を 2 回。いずれも元の位置付近へ戻る。
  ★ 周囲に linear_distance + 0.2 m 以上の空きを確保すること。

並進だけ / 回転だけも測れる (測らなかった方の補正は 1.0 = 現状維持):
    ... -p linear_repeats:=0                       # 回転だけ
    ... -p "rotation_angles_deg:=[0.0]"            # 並進だけ (0° は飛ばす)

★ 回転は 360° 単位にする。その場で 1 周すれば視点が元に戻るので、家具の脚の
  ような疎な環境でも ICP が静止時と同じ精度 (0.01° 程度) で合う。90° などは
  自機の死角が回転とともに動き、重なる点が減る。
★ 並進は視点が変わるぶん重なりが減る。雑然とした部屋では linear_distance を
  0.15 m 程度に抑える (0.30 m では重なりが 6 割を切り ICP が数 cm ずれた)。

真値は slam_toolbox ではなく、静止中に取った生 ``/scan`` 同士の ICP。
slam_toolbox は 0.1 m / 0.1 rad 動くまでスキャンを処理しないので、止まる
直前の動きが map → odom に反映されず、較正の基準にならない。

結果は表で表示し、``output`` (既定 /maps/odom_calibration.json) に保存する。
base.yaml は書き換えない。値を確認してから手で反映すること。
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from lekiwi_base_bringup import scan_matching as sm


class OdomCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("odom_calibrator")
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("linear_distance", 0.15)
        self.declare_parameter("linear_repeats", 2)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("rotation_angles_deg", [360.0, -360.0, 360.0, -360.0])
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("scans_per_capture", 10)
        self.declare_parameter("min_range", 0.35)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("voxel", 0.015)
        self.declare_parameter("max_icp_rms", 0.02)
        self.declare_parameter("min_inlier_ratio", 0.6)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("driver_node", "/lekiwi_base_driver")
        self.declare_parameter("output", "/maps/odom_calibration.json")

        self.odom_pose: tuple[float, float, float] | None = None
        self.odom_yaw_unwrapped = 0.0
        self.scans: list[LaserScan] = []
        self.collecting = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # ── 購読 ──────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.odom_pose is not None:
            self.odom_yaw_unwrapped += sm.wrap_angle(yaw - self.odom_pose[2])
        self.odom_pose = (p.position.x, p.position.y, yaw)

    def _on_scan(self, msg: LaserScan) -> None:
        if self.collecting:
            self.scans.append(msg)

    # ── 補助 ──────────────────────────────────────────────────────────

    def param(self, name: str):
        return self.get_parameter(name).value

    def spin_for(self, seconds: float, cmd: Twist | None = None) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self) -> None:
        zero = Twist()
        for _ in range(10):
            try:
                self.cmd_pub.publish(zero)
            except Exception:  # Ctrl+C 後で context が無効でも、残りの終了処理は続ける
                return
            time.sleep(0.02)

    def sensor_pose(self, frame: str) -> tuple[float, float, float]:
        tf = self.tf_buffer.lookup_transform(
            str(self.param("base_frame")), frame, Time(), timeout=Duration(seconds=5.0)
        )
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def capture(self):
        """静止したまま数枚のスキャンを集め、base 座標の点群と odom 姿勢を返す。"""
        self.stop()
        self.spin_for(float(self.param("settle_s")), Twist())
        self.scans = []
        self.collecting = True
        need = int(self.param("scans_per_capture"))
        deadline = time.monotonic() + 10.0
        while len(self.scans) < need and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collecting = False
        if len(self.scans) < need:
            raise RuntimeError(f"/scan が {len(self.scans)}/{need} 枚しか届かない")
        pose = self.sensor_pose(self.scans[0].header.frame_id)
        points = np.vstack([
            sm.scan_to_points(
                s.ranges, s.angle_min, s.angle_increment,
                float(self.param("min_range")), float(self.param("max_range")), pose,
            )
            for s in self.scans
        ])
        cloud = sm.voxel_downsample(points, float(self.param("voxel")))
        return cloud, self.odom_pose, self.odom_yaw_unwrapped

    def driver_radii(self) -> tuple[float, float]:
        """較正の起点は base.yaml ではなく、いま動いているドライバの値。"""
        client = self.create_client(
            GetParameters, f"{self.param('driver_node')}/get_parameters"
        )
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{self.param('driver_node')} のパラメータを読めない")
        future = client.call_async(
            GetParameters.Request(names=["wheel_radius", "base_radius"])
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            raise RuntimeError("wheel_radius / base_radius の取得がタイムアウト")
        wheel, base = (v.double_value for v in future.result().values)
        return wheel, base

    def move(self, vx: float, vy: float, wz: float, seconds: float) -> None:
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        self.spin_for(seconds, cmd)
        self.stop()

    # ── 本体 ──────────────────────────────────────────────────────────

    def plan(self):
        v = float(self.param("linear_speed"))
        d = float(self.param("linear_distance"))
        w = float(self.param("angular_speed"))
        steps = []
        for i in range(int(self.param("linear_repeats"))):
            steps += [
                (f"前進{i + 1}", "linear", (v, 0.0, 0.0), d / v),
                (f"後退{i + 1}", "linear", (-v, 0.0, 0.0), d / v),
                (f"左{i + 1}", "linear", (0.0, v, 0.0), d / v),
                (f"右{i + 1}", "linear", (0.0, -v, 0.0), d / v),
            ]
        for deg in self.param("rotation_angles_deg"):
            if float(deg) == 0.0:
                continue
            rad = math.radians(float(deg))
            steps.append(
                (f"回転{float(deg):+.0f}°", "rotation", (0.0, 0.0, math.copysign(w, rad)), abs(rad) / w)
            )
        return steps

    def run(self) -> dict:
        wheel_radius, base_radius = self.driver_radii()
        self.get_logger().info(f"現在の wheel_radius={wheel_radius} base_radius={base_radius}")
        self.get_logger().info("/odom と /scan を待っています")
        deadline = time.monotonic() + 15.0
        while self.odom_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom_pose is None:
            raise RuntimeError("/odom が届かない")

        cloud0, pose0, yaw0 = self.capture()
        # 動かさずにもう 1 回取り、ICP の雑音の大きさを先に測る
        cloud_still, _, _ = self.capture()
        still = sm.icp_2d(cloud_still, cloud0)
        self.get_logger().info(
            f"静止での ICP: x={still.pose[0] * 1000:.1f}mm y={still.pose[1] * 1000:.1f}mm "
            f"yaw={math.degrees(still.pose[2]):.2f}° rms={still.rms * 1000:.1f}mm "
            f"inlier={still.inlier_ratio:.2f} 点数={len(cloud0)}"
        )
        self.check_icp("静止", still)

        segments: list[sm.Segment] = []
        rows = []
        for name, kind, (vx, vy, wz), seconds in self.plan():
            self.get_logger().info(f"{name}: v=({vx}, {vy}) wz={wz} を {seconds:.2f}s")
            self.move(vx, vy, wz, seconds)
            cloud1, pose1, yaw1 = self.capture()
            ox, oy, _ = sm.relative_pose(pose0, pose1)
            odom = (ox, oy, yaw1 - yaw0)
            result = sm.icp_2d(cloud1, cloud0, initial=(ox, oy, sm.wrap_angle(odom[2])))
            self.check_icp(name, result)
            truth = (
                result.pose[0], result.pose[1],
                sm.unwrap_to_reference(result.pose[2], odom[2]),
            )
            segments.append(sm.Segment(name, kind, odom, truth))
            rows.append({
                "name": name, "kind": kind, "odom": odom, "truth": truth,
                "icp_rms": result.rms, "icp_inlier_ratio": result.inlier_ratio,
                "icp_min_eigenvalue": result.min_eigenvalue,
            })
            self.get_logger().info(self.format_row(rows[-1]))
            cloud0, pose0, yaw0 = cloud1, pose1, yaw1

        linear_scale, rotation_scale = sm.calibration_scales(segments)
        new_wheel, new_base = sm.calibrated_radii(
            wheel_radius, base_radius, linear_scale, rotation_scale
        )
        return {
            "still_icp": {"pose": still.pose, "rms": still.rms},
            "segments": rows,
            "linear_scale": linear_scale,
            "rotation_scale": rotation_scale,
            "current": {"wheel_radius": wheel_radius, "base_radius": base_radius},
            "suggested": {"wheel_radius": new_wheel, "base_radius": new_base},
        }

    def check_icp(self, name: str, result: sm.IcpResult) -> None:
        if result.rms > float(self.param("max_icp_rms")) or result.inlier_ratio < float(
            self.param("min_inlier_ratio")
        ):
            raise RuntimeError(
                f"{name}: ICP が信用できない (rms={result.rms * 1000:.1f}mm "
                f"inlier={result.inlier_ratio:.2f})。周囲に動く物が無いか、壁が見えているか確認"
            )

    @staticmethod
    def format_row(row: dict) -> str:
        o, t = row["odom"], row["truth"]
        if row["kind"] == "linear":
            od, td = math.hypot(o[0], o[1]), math.hypot(t[0], t[1])
            return (
                f"{row['name']}: odom {od * 100:.2f}cm 真値 {td * 100:.2f}cm "
                f"(差 {(od - td) * 1000:+.1f}mm, 倍率 {td / od:.4f}) "
                f"真値の向き誤差 x={t[0] * 1000:+.1f} y={t[1] * 1000:+.1f}mm "
                f"yaw={math.degrees(t[2]):+.2f}° / odom yaw={math.degrees(o[2]):+.2f}°"
            )
        return (
            f"{row['name']}: odom {math.degrees(o[2]):+.2f}° 真値 {math.degrees(t[2]):+.2f}° "
            f"(倍率 {t[2] / o[2]:.4f}) 並進ずれ 真値 ({t[0] * 1000:+.1f}, {t[1] * 1000:+.1f})mm"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomCalibrator()
    try:
        report = node.run()
        def fmt(v):
            return "未測定" if v is None else f"{v:.4f}"

        node.get_logger().info(
            f"並進の倍率 {fmt(report['linear_scale'])} / 回転の倍率 {fmt(report['rotation_scale'])}"
        )
        node.get_logger().info(
            f"提案: wheel_radius {report['current']['wheel_radius']} → "
            f"{report['suggested']['wheel_radius']:.5f}, base_radius "
            f"{report['current']['base_radius']} → {report['suggested']['base_radius']:.5f}"
        )
        output = str(node.param("output"))
        with open(output, "w") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        node.get_logger().info(f"保存: {output}")
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
    finally:
        # ★ どこで抜けても速度ゼロを送ってから終わる
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
