import pytest

from lekiwi_base_bringup.sts_bus import StsBus, StsBusError


def _bus():
    bus = object.__new__(StsBus)
    bus.ids = list(range(1, 10))
    return bus


def test_selected_ids_defaults_to_whole_bus():
    assert _bus()._selected_ids(None) == list(range(1, 10))


def test_selected_ids_preserves_wheel_subset_order():
    assert _bus()._selected_ids([7, 8, 9]) == [7, 8, 9]


def test_selected_ids_rejects_ids_outside_owned_bus():
    with pytest.raises(ValueError, match="10"):
        _bus()._selected_ids([7, 10])


# ── 位置の同期読み取りの再試行回数 ──────────────────────────────────────


class _FakeSyncReader:
    """``fail_times`` 回だけ失敗し、その後は ID ごとの値を返す。"""

    def __init__(self, fail_times, values):
        self.fail_times = fail_times
        self.values = values
        self.calls = 0

    def txRxPacket(self):
        from lekiwi_base_bringup.sts_bus import scs

        self.calls += 1
        return scs.COMM_RX_TIMEOUT if self.calls <= self.fail_times else scs.COMM_SUCCESS

    def getData(self, motor_id, _address, _length):
        return self.values[motor_id]


class _FakePacket:
    def getTxRxResult(self, result):
        return f"result={result}"


def _position_bus(fail_times):
    bus = object.__new__(StsBus)
    bus.ids = [7, 8, 9]
    bus.num_retry = 5
    bus._port = object()
    bus._packet = _FakePacket()
    bus._sync_position_reader = _FakeSyncReader(fail_times, {7: 10, 8: 4100, 9: 0})
    return bus


def test_read_positions_retries_by_default():
    bus = _position_bus(fail_times=3)
    assert bus.read_positions() == {7: 10, 8: 4, 9: 0}  # 4096 で折り返す
    assert bus._sync_position_reader.calls == 4


def test_read_positions_without_retry_tries_once():
    # 制御周期からの呼び出し。1 回の失敗で諦め、タイムアウト待ちを重ねない
    bus = _position_bus(fail_times=1)
    with pytest.raises(StsBusError, match="同期読み取り失敗"):
        bus.read_positions(retries=0)
    assert bus._sync_position_reader.calls == 1


def test_read_positions_gives_up_after_num_retry():
    bus = _position_bus(fail_times=100)
    with pytest.raises(StsBusError):
        bus.read_positions()
    assert bus._sync_position_reader.calls == 6
