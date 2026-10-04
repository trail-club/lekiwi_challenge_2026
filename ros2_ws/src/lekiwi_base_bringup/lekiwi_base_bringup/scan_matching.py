"""LiDAR スキャンの 2D 位置合わせ (ICP) とオドメトリ較正の計算。

``calibrate_odom`` から ROS 2 を取り除いた部分。numpy しか import しないので
ROS 2 なしで単体テストできる (scipy はイメージに無いので使わない)。

使い方の流れ:

1. 静止中のスキャン数枚を ``scan_to_points`` で base_footprint 座標の点群にし、
   ``voxel_downsample`` で平均化する
2. 動作の前後の点群を ``icp_2d`` に渡し、車体の実移動量 (前姿勢から見た後姿勢)
   を得る
3. エンコーダ odom の移動量と比べ、``calibration_scales`` / ``calibrated_radii`` で
   ``wheel_radius`` と ``base_radius`` の補正値を出す

姿勢はすべて ``(x, y, yaw)``。``yaw`` は rad。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# ── 姿勢 ─────────────────────────────────────────────────────────────────


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose(before, after) -> tuple[float, float, float]:
    """``before`` から見た ``after`` の姿勢 (yaw は折り返す)。"""
    dx, dy = after[0] - before[0], after[1] - before[1]
    c, s = math.cos(before[2]), math.sin(before[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap_angle(after[2] - before[2])


def transform_points(points: np.ndarray, pose) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + np.array([x, y])


# ── 点群 ─────────────────────────────────────────────────────────────────


def scan_to_points(
    ranges,
    angle_min: float,
    angle_increment: float,
    min_range: float,
    max_range: float,
    sensor_pose=(0.0, 0.0, 0.0),
) -> np.ndarray:
    """LaserScan の測距値を ``sensor_pose`` (base 座標でのセンサ姿勢) で変換した点群にする。

    ``min_range`` 未満は捨てる。★ 自機の車輪やボディが映るので、ここを
    十分大きくしないと「一緒に動く点」が位置合わせを移動ゼロ側へ引っ張る。
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(r.size)
    keep = np.isfinite(r) & (r >= min_range) & (r <= max_range)
    pts = np.column_stack([r[keep] * np.cos(angles[keep]), r[keep] * np.sin(angles[keep])])
    return transform_points(pts, sensor_pose)


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """``voxel`` [m] 格子ごとに点を平均する。複数スキャンを重ねた雑音を減らす。"""
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((counts.size, 2))
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def _nearest(query: np.ndarray, target: np.ndarray, chunk: int = 2048):
    """総当たりの最近傍。点数は数千なので KD 木なしで足りる。"""
    idx = np.empty(len(query), dtype=np.int64)
    dist = np.empty(len(query))
    for start in range(0, len(query), chunk):
        q = query[start:start + chunk]
        d2 = ((q[:, None, :] - target[None, :, :]) ** 2).sum(axis=2)
        i = np.argmin(d2, axis=1)
        idx[start:start + chunk] = i
        dist[start:start + chunk] = np.sqrt(d2[np.arange(len(q)), i])
    return idx, dist


def estimate_normals(points: np.ndarray, k: int = 6):
    """各点の法線と平面らしさ (0=直線上, 1=等方的) を近傍 ``k`` 点の主成分で求める。"""
    d2 = ((points[:, None, :] - points[None, :, :]) ** 2).sum(axis=2)
    neighbours = np.argsort(d2, axis=1)[:, :k]
    local = points[neighbours] - points[neighbours].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", local, local) / k
    eigval, eigvec = np.linalg.eigh(cov)  # 昇順
    normals = eigvec[:, :, 0]
    flatness = eigval[:, 0] / np.maximum(eigval[:, 1], 1e-12)
    return normals, flatness


# ── ICP ──────────────────────────────────────────────────────────────────


@dataclass
class IcpResult:
    pose: tuple[float, float, float]
    rms: float  # 採用した対応点の点-直線距離 RMS [m]
    inlier_ratio: float  # 最終しきい値以内に入った source 点の割合
    min_eigenvalue: float  # 正規方程式の最小固有値。小さいと拘束不足 (廊下など)
    iterations: int


def icp_2d(
    source: np.ndarray,
    target: np.ndarray,
    initial=(0.0, 0.0, 0.0),
    max_distances=(0.30, 0.15, 0.08, 0.04),
    iterations_per_stage: int = 15,
    max_flatness: float = 0.3,
) -> IcpResult:
    """``transform_points(source, pose) ≈ target`` となる ``pose`` を点-直線 ICP で求める。

    source を動作後、target を動作前の点群 (どちらも base 座標) にすると、
    ``pose`` は動作前の姿勢から見た動作後の姿勢 = 車体の実移動量になる。
    対応点のしきい値を段階的に絞り、初期値の誤差 (数 cm・数度) を吸収する。
    """
    normals, flatness = estimate_normals(target)
    usable = flatness <= max_flatness
    x, y, yaw = (float(v) for v in initial)
    total_iterations = 0
    rms = math.inf
    inliers = 0.0
    min_eig = 0.0
    for max_distance in max_distances:
        for _ in range(iterations_per_stage):
            total_iterations += 1
            moved = transform_points(source, (x, y, yaw))
            idx, dist = _nearest(moved, target)
            mask = (dist < max_distance) & usable[idx]
            if mask.sum() < 10:
                raise ValueError(f"対応点が足りない ({int(mask.sum())} 点)。初期値かスキャンを確認")
            p = moved[mask]
            q = target[idx[mask]]
            n = normals[idx[mask]]
            residual = ((p - q) * n).sum(axis=1)
            # 現在の点群に対する微小変換 (dx, dy, dθ) で線形化する
            jac = np.column_stack([n[:, 0], n[:, 1], n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1]])
            hessian = jac.T @ jac
            delta = np.linalg.solve(hessian, -jac.T @ residual)
            c, s = math.cos(delta[2]), math.sin(delta[2])
            x, y = c * x - s * y + delta[0], s * x + c * y + delta[1]
            yaw = wrap_angle(yaw + delta[2])
            rms = float(np.sqrt(np.mean(residual ** 2)))
            inliers = float(mask.mean())
            min_eig = float(np.linalg.eigvalsh(hessian / mask.sum())[0])
            if abs(delta[0]) < 1e-5 and abs(delta[1]) < 1e-5 and abs(delta[2]) < 1e-5:
                break
    return IcpResult((x, y, yaw), rms, inliers, min_eig, total_iterations)


def unwrap_to_reference(angle: float, reference: float) -> float:
    """折り返された ``angle`` に 2π の整数倍を足し、``reference`` に最も近くする。

    360° 回転の真値は ICP では 0° 付近に見えるので、odom の積算角を参照に戻す。
    """
    return angle + 2.0 * math.pi * round((reference - angle) / (2.0 * math.pi))


# ── 較正 ─────────────────────────────────────────────────────────────────


@dataclass
class Segment:
    """1 動作ぶんの比較。``kind`` は ``"linear"`` か ``"rotation"``。"""

    name: str
    kind: str
    odom: tuple[float, float, float]  # yaw は積算角 (折り返さない)
    truth: tuple[float, float, float]  # yaw は odom に合わせて展開済み


def calibration_scales(segments) -> tuple[float | None, float | None]:
    """``(並進の倍率, 回転の倍率)`` = 真値 / odom。距離・角度の重み付き平均。

    片方の区間しか無ければ、無い方は ``None``。並進と回転は別々に測ってよい
    (回転はその場で視点が変わらないので、雑然とした部屋でも精度が出る)。
    """
    lin = [s for s in segments if s.kind == "linear"]
    rot = [s for s in segments if s.kind == "rotation"]
    if not lin and not rot:
        raise ValueError("区間が 1 つも無い")
    linear_scale = None
    rotation_scale = None
    if lin:
        linear_scale = sum(math.hypot(s.truth[0], s.truth[1]) for s in lin) / sum(
            math.hypot(s.odom[0], s.odom[1]) for s in lin
        )
    if rot:
        rotation_scale = sum(abs(s.truth[2]) for s in rot) / sum(abs(s.odom[2]) for s in rot)
    return linear_scale, rotation_scale


def calibrated_radii(
    wheel_radius: float,
    base_radius: float,
    linear_scale: float | None,
    rotation_scale: float | None,
) -> tuple[float, float]:
    """倍率から新しい ``(wheel_radius, base_radius)`` を求める。

    エンコーダ odom の並進速度は ``wheel_radius`` に比例し、角速度は
    ``wheel_radius / base_radius`` に比例する。並進の倍率で車輪半径を直し、
    残りの回転の倍率を base_radius で吸収する。測っていない方 (``None``) は
    1.0 として扱う (= その成分は今のまま)。
    """
    linear_scale = 1.0 if linear_scale is None else linear_scale
    rotation_scale = 1.0 if rotation_scale is None else rotation_scale
    new_wheel = wheel_radius * linear_scale
    new_base = base_radius * linear_scale / rotation_scale
    return new_wheel, new_base
