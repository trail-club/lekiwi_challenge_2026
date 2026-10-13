"""scan_filter が使うスキャン前処理 (rclpy に依存しない純粋関数)。

``scan_filter.py`` は rclpy を import するので ROS 2 の無いホストでは
テストできない。前処理の中身だけをここに切り出して ``test/`` から検証する。

  * mask_outside_intervals: 有効区間の外を inf にする (機体が映る角度を消す)
  * interpolate_short_gaps: 短い欠損 (inf / nan) を両隣から線形補完する
"""

from __future__ import annotations

import math
from collections.abc import Sequence


def parse_intervals_deg(flat: Sequence[float]) -> list[tuple[float, float]]:
    """[min1, max1, min2, max2, ...] [deg] を [(min1, max1), ...] [rad] にする。

    ROS 2 のパラメータは入れ子の配列を持てないので平たい配列で受け取る。
    min > max の区間は ±180° をまたぐ区間とみなす (lidar_angle が末尾と
    先頭をつなげた区間を出すとこうなる)。
    """
    if len(flat) == 0 or len(flat) % 2 != 0:
        raise ValueError(f"有効区間は [min, max] の組で指定する: {list(flat)}")
    return [
        (math.radians(flat[k]), math.radians(flat[k + 1]))
        for k in range(0, len(flat), 2)
    ]


def normalize_angle(angle: float) -> float:
    """-π 〜 π に正規化する。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def in_intervals(angle: float, intervals: Sequence[tuple[float, float]]) -> bool:
    """angle [rad] がどれかの区間に入っていれば True。両端を含む。"""
    a = normalize_angle(angle)
    for lo, hi in intervals:
        if lo <= hi:
            if lo <= a <= hi:
                return True
        # ±180° をまたぐ区間
        elif a >= lo or a <= hi:
            return True
    return False


def mask_outside_intervals(
    ranges: list[float],
    angle_min: float,
    angle_increment: float,
    intervals: Sequence[tuple[float, float]],
) -> list[bool]:
    """有効区間の外の点を inf に置き換える (ranges をその場で書き換える)。

    各点が有効区間の中かどうかを返す (interpolate_short_gaps の inside に渡す)。
    """
    inside = [in_intervals(angle_min + i * angle_increment, intervals) for i in range(len(ranges))]
    for i, ok in enumerate(inside):
        if not ok:
            ranges[i] = float("inf")
    return inside


def interpolate_short_gaps(
    ranges: list[float],
    max_gap: int,
    range_min: float,
    range_max: float,
    inside: Sequence[bool] | None = None,
) -> None:
    """max_gap ビーム以下の欠損を両隣の値で線形補完する (ranges をその場で書き換える)。

    欠損は inf / nan の点。両隣が有効 (有限かつ range_min 〜 range_max) な
    ときだけ補完する。片側が端・欠損・範囲外なら触らない。

    inside (mask_outside_intervals の戻り値) を渡すと、マスクした点を含む欠損は
    補完しない。渡さないと、2 ビーム以下の狭いマスクが両隣から埋め戻される。
    マスクした点は inf なので、機体の点や区間の外の値が隣として使われることは無い。

    先頭と末尾はつなげない。一周ぶんの LiDAR でも ±180° は後輪の方向で
    マスクされるので、つなげても補完される点は無い。
    """
    if max_gap <= 0:
        return

    def valid(r: float) -> bool:
        return math.isfinite(r) and range_min <= r <= range_max

    n = len(ranges)
    i = 0
    while i < n:
        if math.isfinite(ranges[i]):
            i += 1
            continue
        # ranges[i:j] が欠損の連続
        j = i
        while j < n and not math.isfinite(ranges[j]):
            j += 1
        gap = j - i
        fillable = (
            gap <= max_gap
            and i > 0
            and j < n
            and valid(ranges[i - 1])
            and valid(ranges[j])
            and (inside is None or all(inside[i:j]))
        )
        if fillable:
            left, right = ranges[i - 1], ranges[j]
            for k in range(gap):
                t = (k + 1) / (gap + 1)
                ranges[i + k] = left + (right - left) * t
        i = j
