"""車輪位置エンコーダの差分追跡。

``base_driver`` から ROS 2 とシリアルを取り除いた部分。numpy しか import しない
ので、``kinematics`` と同じく ROS 2 なしで単体テストできる。

責務は 3 つだけ。

1. 前回の位置 (基準) と時刻を持ち、今回の位置との周回補正済み差分を返す
2. サンプル間隔が長すぎて周回方向を一意に決められない区間を「積分しない」と判定する
3. 最後に読めた時刻から、欠測が続いているか (= 停止すべきか) を判定する

時刻はすべて秒の float。ROS の ``Time`` への変換は呼び出し側で行う。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from lekiwi_base_bringup import kinematics as kin

#: 1 サンプル間の移動を半回転より十分小さく保つための安全係数。
#: 1.0 だと最高速度でちょうど半回転になり、折り返しの判定が境界で曖昧になる。
ALIAS_SAFETY = 0.5


def max_unambiguous_dt(max_abs_ticks: float, safety: float = ALIAS_SAFETY) -> float:
    """``max_abs_ticks`` [ticks/s] で回る車輪の周回方向を決められる最大の間隔 [s]。"""
    return safety * kin.TICKS_PER_REV / (2.0 * max(1.0, abs(float(max_abs_ticks))))


@dataclass
class EncoderSample:
    """1 回の読み取り結果。``integrable`` が False の区間は積分しない。"""

    dt: float
    deltas: np.ndarray
    integrable: bool

    @property
    def ticks_per_s(self) -> np.ndarray:
        return self.deltas / self.dt


class EncoderTracker:
    """位置エンコーダの基準値と最終成功時刻を持つ。"""

    def __init__(self) -> None:
        self.positions: np.ndarray | None = None
        self.last_success_s: float | None = None
        self.failures = 0

    def reset(self, positions, now_s: float) -> None:
        """基準を ``positions`` に置き直す。この時点までの移動は積分しない。"""
        self.positions = np.asarray(positions, dtype=float)
        self.last_success_s = now_s
        self.failures = 0

    def restamp(self, now_s: float) -> None:
        """基準位置はそのままに時刻だけ進める (車輪が止まっていると分かっている間用)。"""
        self.last_success_s = now_s

    def record_failure(self) -> int:
        """読み取り失敗を記録する。基準は保持し、次の成功時にまとめて差分を取る。"""
        self.failures += 1
        return self.failures

    def update(self, positions, now_s: float, max_abs_ticks: float) -> EncoderSample | None:
        """読み取りに成功した位置を取り込む。

        基準が無ければ基準を作って ``None`` を返す。基準があれば、積分の可否に
        かかわらず基準を今回値へ進める。区間が長すぎた場合も、ここで基準が
        置き直されるので次の周期から正常に積分が再開する。
        """
        if self.positions is None or self.last_success_s is None:
            self.reset(positions, now_s)
            return None
        positions = np.asarray(positions, dtype=float)
        dt = now_s - self.last_success_s
        deltas = kin.encoder_position_deltas(positions, self.positions)
        self.reset(positions, now_s)
        integrable = 0.0 < dt < max_unambiguous_dt(max_abs_ticks)
        return EncoderSample(dt=dt, deltas=deltas, integrable=integrable)

    def seconds_since_success(self, now_s: float) -> float:
        if self.last_success_s is None:
            return math.inf
        return now_s - self.last_success_s

    def is_stale(self, now_s: float, timeout_s: float) -> bool:
        """最後に読めてから ``timeout_s`` を超えたか。超えたら車輪を止める。"""
        return self.seconds_since_success(now_s) > timeout_s
