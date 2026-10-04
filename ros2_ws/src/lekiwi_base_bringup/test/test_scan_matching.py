"""``lekiwi_base_bringup.scan_matching`` の単体テスト。

実機も ROS 2 も不要。合成した部屋のスキャンで、ICP が既知の移動量を
取り戻せることと、較正の計算 (倍率 → 半径) を固定する。

実行 (ROS 2 なし):
    cd ros2_ws/src/lekiwi_base_bringup
    uv run --no-project --with pytest --with numpy pytest test/test_scan_matching.py -v
"""

import math

import numpy as np
import pytest

from lekiwi_base_bringup import scan_matching as sm

# 4 m × 3 m の部屋 + 柱 2 本。対称だと位置が決まらないので、わざと非対称にする
WALLS = [
    ((-1.5, -1.2), (2.5, -1.2)),
    ((2.5, -1.2), (2.5, 1.8)),
    ((2.5, 1.8), (-1.5, 1.8)),
    ((-1.5, 1.8), (-1.5, -1.2)),
    ((0.8, 0.6), (1.2, 0.6)),
    ((1.2, 0.6), (1.2, 1.0)),
    ((-0.9, -0.6), (-0.6, -0.6)),
    ((-0.6, -0.6), (-0.6, -0.2)),
]


def _raycast(pose, n_beams=720, noise=0.0, rng=None):
    """``pose`` (部屋座標) から見た LaserScan 相当の ``(ranges, angle_min, inc)``。"""
    x, y, yaw = pose
    angles = -math.pi + 2 * math.pi * np.arange(n_beams) / n_beams
    ranges = np.full(n_beams, np.inf)
    for i, a in enumerate(angles):
        d = np.array([math.cos(yaw + a), math.sin(yaw + a)])
        for (ax, ay), (bx, by) in WALLS:
            e = np.array([bx - ax, by - ay])
            m = np.array([[d[0], -e[0]], [d[1], -e[1]]])
            if abs(np.linalg.det(m)) < 1e-9:
                continue
            t, u = np.linalg.solve(m, np.array([ax - x, ay - y]))
            if t > 0 and 0 <= u <= 1:
                ranges[i] = min(ranges[i], t)
    if noise:
        ranges = ranges + rng.normal(0, noise, n_beams)
    return ranges, -math.pi, 2 * math.pi / n_beams


def _cloud(pose, scans=1, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    pts = []
    for _ in range(scans):
        r, a0, inc = _raycast(pose, noise=noise, rng=rng)
        pts.append(sm.scan_to_points(r, a0, inc, 0.35, 8.0))
    return sm.voxel_downsample(np.vstack(pts), 0.015)


# ── 姿勢の補助 ──────────────────────────────────────────────────────────


def test_relative_pose_in_rotated_frame() -> None:
    x, y, yaw = sm.relative_pose((1.0, 1.0, math.pi / 2), (1.0, 1.3, math.pi / 2 + 0.1))
    assert (x, y) == pytest.approx((0.3, 0.0), abs=1e-12)
    assert yaw == pytest.approx(0.1)


def test_unwrap_to_reference_recovers_full_turn() -> None:
    assert sm.unwrap_to_reference(0.02, 2 * math.pi - 0.05) == pytest.approx(2 * math.pi + 0.02)
    assert sm.unwrap_to_reference(-0.02, -2 * math.pi) == pytest.approx(-2 * math.pi - 0.02)
    assert sm.unwrap_to_reference(1.5, 1.4) == pytest.approx(1.5)


def test_scan_to_points_drops_near_returns_and_applies_sensor_pose() -> None:
    pts = sm.scan_to_points([0.2, 1.0, np.inf], 0.0, math.pi / 2, 0.35, 8.0, (0.1, 0.0, 0.0))
    # 0.2 m (自機) と inf は捨て、90° 方向 1 m の点だけ残る
    np.testing.assert_allclose(pts, [[0.1, 1.0]], atol=1e-12)


def test_voxel_downsample_averages_within_cell() -> None:
    pts = np.array([[0.001, 0.001], [0.009, 0.009], [0.5, 0.5]])
    out = sm.voxel_downsample(pts, 0.01)
    assert len(out) == 2
    assert any(np.allclose(p, [0.005, 0.005]) for p in out)


# ── ICP ─────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "motion",
    [
        (0.30, 0.0, 0.0),  # 前進
        (0.0, -0.30, 0.0),  # 右
        (0.0, 0.0, math.radians(90)),  # 純回転
        (0.05, 0.02, math.radians(-30)),
    ],
)
def test_icp_recovers_known_motion(motion) -> None:
    start = (0.2, 0.1, math.radians(10))
    c, s = math.cos(start[2]), math.sin(start[2])
    end = (
        start[0] + c * motion[0] - s * motion[1],
        start[1] + s * motion[0] + c * motion[1],
        start[2] + motion[2],
    )
    before, after = _cloud(start), _cloud(end)
    # odom 相当の初期値は 3% ずらす (較正前のスケール誤差の想定)
    initial = tuple(v * 1.03 for v in motion)
    result = sm.icp_2d(after, before, initial=initial)
    assert result.pose[0] == pytest.approx(motion[0], abs=2e-3)
    assert result.pose[1] == pytest.approx(motion[1], abs=2e-3)
    assert math.degrees(result.pose[2]) == pytest.approx(math.degrees(motion[2]), abs=0.1)
    assert result.inlier_ratio > 0.9


def test_icp_with_noise_averages_multiple_scans() -> None:
    # RPLIDAR A1 程度 (σ=1 cm) の雑音でも、10 枚重ねれば mm 単位に収まる
    start, motion = (0.0, 0.0, 0.0), (0.30, 0.0, 0.0)
    before = _cloud(start, scans=10, noise=0.01, seed=1)
    after = _cloud((0.30, 0.0, 0.0), scans=10, noise=0.01, seed=2)
    result = sm.icp_2d(after, before, initial=(0.29, 0.0, 0.0))
    assert result.pose[0] == pytest.approx(motion[0], abs=3e-3)
    assert result.pose[1] == pytest.approx(0.0, abs=3e-3)
    assert math.degrees(result.pose[2]) == pytest.approx(0.0, abs=0.2)
    assert result.rms < 0.01


def test_icp_raises_when_clouds_do_not_overlap() -> None:
    before = _cloud((0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="対応点"):
        sm.icp_2d(before + 10.0, before)


# ── 較正 ────────────────────────────────────────────────────────────────


def _segments(linear_scale, rotation_scale):
    return [
        sm.Segment("前進", "linear", (0.30, 0.0, 0.0), (0.30 * linear_scale, 0.0, 0.0)),
        sm.Segment("左", "linear", (0.0, 0.30, 0.0), (0.0, 0.30 * linear_scale, 0.0)),
        sm.Segment(
            "回転", "rotation", (0.0, 0.0, 2 * math.pi), (0.0, 0.0, 2 * math.pi * rotation_scale)
        ),
        sm.Segment(
            "逆回転", "rotation", (0.0, 0.0, -math.pi / 2),
            (0.0, 0.0, -math.pi / 2 * rotation_scale),
        ),
    ]


def test_calibration_scales_are_truth_over_odom() -> None:
    lin, rot = sm.calibration_scales(_segments(1.02, 0.98))
    assert lin == pytest.approx(1.02)
    assert rot == pytest.approx(0.98)


def test_calibration_with_one_kind_leaves_other_none() -> None:
    assert sm.calibration_scales(_segments(1.02, 0.98)[:2]) == (pytest.approx(1.02), None)
    assert sm.calibration_scales(_segments(1.02, 0.98)[2:]) == (None, pytest.approx(0.98))
    with pytest.raises(ValueError):
        sm.calibration_scales([])


def test_rotation_only_calibration_changes_base_radius_only() -> None:
    new_r, new_big_r = sm.calibrated_radii(0.05, 0.125, None, 0.98)
    assert new_r == pytest.approx(0.05)
    assert (new_r / new_big_r) / (0.05 / 0.125) == pytest.approx(0.98)


def test_calibrated_radii_restore_true_motion() -> None:
    # 並進は r に、回転は r/R に比例する。補正後の odom が真値と一致すること
    r, big_r = 0.05, 0.125
    lin, rot = 1.02, 0.97
    new_r, new_big_r = sm.calibrated_radii(r, big_r, lin, rot)
    assert new_r / r == pytest.approx(lin)
    assert (new_r / new_big_r) / (r / big_r) == pytest.approx(rot)


def test_calibrated_radii_identity() -> None:
    assert sm.calibrated_radii(0.05, 0.125, 1.0, 1.0) == pytest.approx((0.05, 0.125))
