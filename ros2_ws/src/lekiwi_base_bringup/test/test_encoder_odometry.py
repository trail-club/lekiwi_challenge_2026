"""``lekiwi_base_bringup.encoder_odometry`` の単体テスト。

実機もサーボも ROS 2 も不要。``base_driver`` の encoder odometry のうち、
誤積算と暴走に直結する判定 (基準の保持・周回方向の保証・欠測時の停止) を固定する。

実行 (ROS 2 なし):
    cd ros2_ws/src/lekiwi_base_bringup
    uv run --no-project --with pytest --with numpy pytest test/test_encoder_odometry.py -v
"""

import math

import numpy as np
import pytest

from lekiwi_base_bringup import kinematics as kin
from lekiwi_base_bringup.encoder_odometry import (
    ALIAS_SAFETY,
    EncoderTracker,
    max_unambiguous_dt,
)

MAX_TICKS = 3000  # base.yaml の既定値
PERIOD = 0.02  # 50 Hz


def _tracker(positions=(100, 200, 300), now_s=0.0) -> EncoderTracker:
    tracker = EncoderTracker()
    tracker.reset(positions, now_s)
    return tracker


# ── 周回方向を保証できる間隔 ────────────────────────────────────────────


def test_max_unambiguous_dt_has_safety_margin() -> None:
    # 最高速度で 1 区間に進むのは半回転の ALIAS_SAFETY 倍まで
    dt = max_unambiguous_dt(MAX_TICKS)
    assert MAX_TICKS * dt == pytest.approx(ALIAS_SAFETY * kin.TICKS_PER_REV / 2.0)
    assert ALIAS_SAFETY < 1.0


def test_max_unambiguous_dt_covers_control_period() -> None:
    # 50 Hz の通常周期は数周期ぶんの欠測を挟んでも積分できる
    assert max_unambiguous_dt(MAX_TICKS) > 5 * PERIOD


def test_max_unambiguous_dt_shrinks_for_faster_wheels() -> None:
    # 診断モードの生 tick が max_ticks を超えたら、判定も厳しくなる
    assert max_unambiguous_dt(6000) == pytest.approx(max_unambiguous_dt(MAX_TICKS) / 2)
    assert max_unambiguous_dt(-6000) == max_unambiguous_dt(6000)


# ── 通常の更新 ──────────────────────────────────────────────────────────


def test_update_without_baseline_only_sets_baseline() -> None:
    tracker = EncoderTracker()
    assert tracker.update([1, 2, 3], 0.0, MAX_TICKS) is None
    np.testing.assert_array_equal(tracker.positions, [1, 2, 3])


def test_update_returns_deltas_and_velocity() -> None:
    tracker = _tracker()
    sample = tracker.update([110, 190, 320], PERIOD, MAX_TICKS)
    assert sample.integrable
    assert sample.dt == pytest.approx(PERIOD)
    np.testing.assert_array_equal(sample.deltas, [10, -10, 20])
    np.testing.assert_allclose(sample.ticks_per_s, [500, -500, 1000])
    np.testing.assert_array_equal(tracker.positions, [110, 190, 320])


def test_update_handles_wrap_around() -> None:
    tracker = _tracker(positions=(4090, 5, 0))
    sample = tracker.update([5, 4090, 0], PERIOD, MAX_TICKS)
    np.testing.assert_array_equal(sample.deltas, [11, -11, 0])


# ── 読み取り失敗 ────────────────────────────────────────────────────────


def test_failure_keeps_baseline_and_next_success_integrates_whole_gap() -> None:
    tracker = _tracker()
    tracker.record_failure()
    tracker.record_failure()
    np.testing.assert_array_equal(tracker.positions, [100, 200, 300])
    assert tracker.failures == 2

    # 3 周期ぶんの移動を 1 回で拾う。移動量は欠落しない
    sample = tracker.update([160, 140, 300], 3 * PERIOD, MAX_TICKS)
    assert sample.integrable
    assert sample.dt == pytest.approx(3 * PERIOD)
    np.testing.assert_array_equal(sample.deltas, [60, -60, 0])
    assert tracker.failures == 0


def test_long_gap_is_not_integrated_and_baseline_is_reset() -> None:
    tracker = _tracker()
    long_gap = max_unambiguous_dt(MAX_TICKS) + PERIOD
    sample = tracker.update([2000, 200, 300], long_gap, MAX_TICKS)
    assert not sample.integrable

    # 基準は今回値へ置き直され、次の周期から通常どおり積分が再開する
    np.testing.assert_array_equal(tracker.positions, [2000, 200, 300])
    resumed = tracker.update([2010, 200, 300], long_gap + PERIOD, MAX_TICKS)
    assert resumed.integrable
    np.testing.assert_array_equal(resumed.deltas, [10, 0, 0])


def test_gap_limit_depends_on_max_abs_ticks() -> None:
    gap = 0.75 * max_unambiguous_dt(MAX_TICKS)
    assert _tracker().update([100, 200, 300], gap, MAX_TICKS).integrable
    # 倍速の生 tick では同じ間隔でも周回方向を保証できない
    assert not _tracker().update([100, 200, 300], gap, 2 * MAX_TICKS).integrable


@pytest.mark.parametrize("dt", [0.0, -PERIOD])
def test_non_positive_dt_is_not_integrated(dt) -> None:
    assert not _tracker().update([110, 200, 300], dt, MAX_TICKS).integrable


# ── 欠測時の停止判定 ────────────────────────────────────────────────────


def test_stale_after_timeout_without_success() -> None:
    tracker = _tracker(now_s=10.0)
    tracker.record_failure()
    assert not tracker.is_stale(10.4, 0.5)
    assert tracker.is_stale(10.6, 0.5)


def test_success_clears_stale() -> None:
    tracker = _tracker(now_s=10.0)
    assert tracker.is_stale(11.0, 0.5)
    tracker.update([100, 200, 300], 11.0, MAX_TICKS)
    assert not tracker.is_stale(11.1, 0.5)


def test_tracker_without_baseline_is_stale() -> None:
    assert EncoderTracker().is_stale(0.0, 0.5)


def test_restamp_keeps_positions_and_clears_stale() -> None:
    tracker = _tracker(now_s=0.0)
    tracker.restamp(5.0)
    np.testing.assert_array_equal(tracker.positions, [100, 200, 300])
    assert not tracker.is_stale(5.1, 0.5)


# ── /joint_states の車輪角 ──────────────────────────────────────────────


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_position_counts_to_rad_matches_velocity_convention(sign) -> None:
    # n ticks/s で 1 秒回った角度と、n カウントの位置差は一致する
    for counts in (1, -37, 4096):
        assert kin.position_counts_to_rad(counts, sign) == pytest.approx(
            kin.ticks_to_radps(counts, sign) * 1.0
        )


def test_position_counts_to_rad_full_turn() -> None:
    assert kin.position_counts_to_rad(4096) == pytest.approx(2.0 * math.pi)
    assert kin.position_counts_to_rad(4096, -1.0) == pytest.approx(-2.0 * math.pi)
