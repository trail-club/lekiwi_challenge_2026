"""``lekiwi_base_bringup.kinematics`` の単体テスト。

実機もサーボも ROS 2 も不要。``kinematics.py`` は numpy しか import しないため、
Phase D (実機での回転方向・前方向・鏡像の確定) の**前に**運動学だけを切り離して
検証できる。実機で「向きがおかしい」となったとき、原因が運動学なのか配線・符号
なのかを切り分ける土台になる。

ここで固定しているのは主に次の 4 点。

1. ``body → ticks → body`` が整数丸めの範囲で往復すること
   (オープンループ odom はこの往復結果を積分するので、ここが狂うと odom も狂う)
2. 飽和時に **3 輪が比例縮小され進行方向が保たれる** こと
   (1 輪だけ飽和させると合成速度の向きが変わり、高速時に想定外の方向へ走る)
3. README / base.yaml に書かれた **到達可能な速度の数値** が実際に出ること
4. README の Phase D 手順が前提にしている **符号の規約** がコード上も成立すること

実行 (ROS 2 なし):
    cd ros2_ws/src/lekiwi_base_bringup
    uv run --no-project --with pytest --with numpy pytest test/ -v
"""

import itertools
import math

import numpy as np
import pytest

from lekiwi_base_bringup import kinematics as kin

# base.yaml の既定値。テストはこの機体構成を前提にする。
BASE_RADIUS = 0.125
WHEEL_RADIUS = 0.05
ANGLE_OFFSET_DEG = -90.0
MAX_TICKS = 3000

# 車輪の並び順は全体を通して left(ID 7), back(ID 8), right(ID 9)。
LEFT, BACK, RIGHT = 0, 1, 2


@pytest.fixture
def m() -> np.ndarray:
    return kin.wheel_matrix(BASE_RADIUS, ANGLE_OFFSET_DEG)


@pytest.fixture
def m_inv(m: np.ndarray) -> np.ndarray:
    return np.linalg.inv(m)


def to_ticks(vx, vy, wz_degps, m, **kwargs):
    return kin.body_to_wheel_ticks(
        vx, vy, wz_degps, m, WHEEL_RADIUS, max_ticks=MAX_TICKS, **kwargs
    )


# ── 行列の性質 ──────────────────────────────────────────────────────────


def test_wheel_matrix_shape_and_third_column(m: np.ndarray) -> None:
    """第3列は全行 base_radius。純回転で3輪が同じ値になる根拠。"""
    assert m.shape == (3, 3)
    np.testing.assert_allclose(m[:, 2], BASE_RADIUS)


@pytest.mark.parametrize("offset", [0.0, -90.0, 90.0, -180.0, 180.0, 37.5])
def test_wheel_matrix_always_invertible(offset: float) -> None:
    """どの angle_offset でも可逆であること。

    ``base_driver.__init__`` は ``np.linalg.inv(self.m)`` を無条件に呼ぶので、
    特異になる offset が存在すると起動時に例外で落ちる。README が Phase D2 で
    試すよう指示している値 (0 / -90 / 90 / -180) は全て安全でなければならない。

    車輪は常に 120° 間隔なので det は offset に対して回転不変になる。
    """
    matrix = kin.wheel_matrix(BASE_RADIUS, offset)
    det = float(np.linalg.det(matrix))
    assert abs(det) > 1e-6, f"offset={offset} で特異行列になった (det={det})"
    # 条件数が悪化していないことも見る (悪条件だと逆算 odom がノイズを増幅する)
    assert np.linalg.cond(matrix) < 100.0


# ── 往復 (ラウンドトリップ) ─────────────────────────────────────────────


# 1 tick = 360/4096 deg/s の車輪角速度。これが往復誤差の下限を決める。
#   車体並進: (1 tick の車輪周速度) / |m の並進成分| ≒ 1e-4 m/s
#   車体回転: 車輪 deg/s を (base_radius/wheel_radius = 2.5) で割るので
#             1 tick あたり約 0.035 deg/s、丸めで最大その半分の誤差が出る
ROUND_TRIP_ATOL_MPS = 1e-3
ROUND_TRIP_ATOL_DEGPS = 0.05


@pytest.mark.parametrize(
    "vx, vy, wz_degps",
    [
        (0.0, 0.0, 0.0),
        (0.1, 0.0, 0.0),
        (0.0, 0.1, 0.0),
        (0.0, 0.0, 30.0),
        (0.1, 0.05, 15.0),
        (-0.08, -0.05, -20.0),
        (0.26, 0.0, 0.0),  # base.yaml の max_linear_x
        (0.0, 0.0, 103.0),  # base.yaml の max_angular_z 相当 (1.8 rad/s)
    ],
)
def test_round_trip_within_limits(vx, vy, wz_degps, m, m_inv) -> None:
    """飽和しない範囲では body → ticks → body が往復する。

    誤差は 1 tick の量子化由来のみ。odom はこの往復結果を積分するため、ここが
    ずれるとそのまま自己位置のスケール誤差になる。

    ケースは意図的に飽和しないものだけを選んである (飽和込みの検証は
    ``test_saturation_preserves_direction`` の担当)。前提が崩れたら気付けるよう、
    飽和していないことを先に assert している。
    """
    ticks = to_ticks(vx, vy, wz_degps, m)
    assert max(abs(t) for t in ticks) < MAX_TICKS, "この入力は飽和しない想定"

    act_vx, act_vy, act_wz = kin.wheel_ticks_to_body(ticks, m_inv, WHEEL_RADIUS)

    assert act_vx == pytest.approx(vx, abs=ROUND_TRIP_ATOL_MPS)
    assert act_vy == pytest.approx(vy, abs=ROUND_TRIP_ATOL_MPS)
    assert act_wz == pytest.approx(wz_degps, abs=ROUND_TRIP_ATOL_DEGPS)


@pytest.mark.parametrize(
    "signs",
    [
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, -1.0],
        [-1.0, -1.0, -1.0],
    ],
)
def test_direction_signs_cancel_in_round_trip(signs, m, m_inv) -> None:
    """``wheel_direction_signs`` は往復で打ち消される。

    これは「符号は配線の向きを補正するだけで運動学そのものは変えない」という
    設計意図の裏付け。Phase D1 でどう転んでも odom は正しく出る。
    """
    signs_arr = np.asarray(signs, dtype=float)
    vx, vy, wz_degps = 0.12, -0.06, 25.0

    ticks = to_ticks(vx, vy, wz_degps, m, direction_signs=signs_arr)
    act_vx, act_vy, act_wz = kin.wheel_ticks_to_body(
        ticks, m_inv, WHEEL_RADIUS, direction_signs=signs_arr
    )

    assert act_vx == pytest.approx(vx, abs=1e-3)
    assert act_vy == pytest.approx(vy, abs=1e-3)
    assert act_wz == pytest.approx(wz_degps, abs=1e-2)


# ── 飽和 ────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "vx, vy, wz_degps",
    [
        (10.0, 0.0, 0.0),
        (0.0, 10.0, 0.0),
        (0.0, 0.0, 2000.0),
        (5.0, 3.0, 500.0),
        (-8.0, 4.0, -900.0),
    ],
)
def test_saturation_respects_max_ticks(vx, vy, wz_degps, m) -> None:
    """どんな過大入力でも max_ticks を超えない。"""
    ticks = to_ticks(vx, vy, wz_degps, m)
    assert max(abs(t) for t in ticks) <= MAX_TICKS
    # 比例縮小なので、必ずどれか1輪が上限に張り付く
    assert max(abs(t) for t in ticks) == MAX_TICKS


@pytest.mark.parametrize(
    "vx, vy, wz_degps",
    [
        (10.0, 0.0, 0.0),
        (5.0, 3.0, 0.0),
        (-8.0, 4.0, -900.0),
        (1.0, 1.0, 100.0),
        # 上限をわずかに超えるだけのケース。指令値としては現実的な範囲なので、
        # 実運用で最も踏みやすい。
        (0.2, -0.1, 60.0),
        (-0.15, -0.08, -45.0),
    ],
)
def test_saturation_preserves_direction(vx, vy, wz_degps, m, m_inv) -> None:
    """★ 飽和しても進行方向が変わらないこと。

    ``kinematics.py`` の比例縮小 (3輪すべてを同じ係数で縮める) が効いていれば、
    逆算した (vx, vy, wz) は元の指令と **同じ向きで大きさだけ小さい** ベクトルに
    なる。1輪だけクリップする実装に変えるとこのテストが落ちる。
    """
    ticks = to_ticks(vx, vy, wz_degps, m)
    act_vx, act_vy, act_wz = kin.wheel_ticks_to_body(ticks, m_inv, WHEEL_RADIUS)

    want = np.array([vx, vy, math.radians(wz_degps)])
    got = np.array([act_vx, act_vy, math.radians(act_wz)])

    # 縮小されていること (そうでなければ飽和していない = テストの前提が崩れる)
    assert np.linalg.norm(got) < np.linalg.norm(want)

    cosine = float(
        np.dot(want, got) / (np.linalg.norm(want) * np.linalg.norm(got))
    )
    assert cosine == pytest.approx(1.0, abs=1e-4), f"進行方向が変わった (cos={cosine})"


# ── Phase D の手順が前提にしている符号の規約 ────────────────────────────


def test_pure_rotation_drives_all_wheels_equally(m) -> None:
    """純回転では3輪が同じ tick になる。

    README 6.2 D1 の「3輪のリムは同じ向きに回るはず」という判定基準の根拠。
    ここが崩れると D1 の手順自体が意味を失う。
    """
    ticks = to_ticks(0.0, 0.0, 30.0, m)
    assert ticks[LEFT] == ticks[BACK] == ticks[RIGHT]
    assert ticks[LEFT] > 0


def test_forward_does_not_drive_back_wheel(m) -> None:
    """前進 (+x) では後輪 (ID 8) が回らない。

    back の方位角 180° + offset(-90°) = 90° なので cos 成分が 0 になる。
    README 6.2 D2 の「後輪の反対側へ進むのが正しい」の裏付け。
    """
    assert m[BACK, 0] == pytest.approx(0.0, abs=1e-12)

    ticks = to_ticks(0.15, 0.0, 0.0, m)
    assert ticks[BACK] == 0
    # 左右輪は逆向きに回って前進成分を作る
    assert ticks[LEFT] * ticks[RIGHT] < 0


def test_left_translation_sign_of_back_wheel(m) -> None:
    """+y (左) 平行移動で後輪が負に回る。

    README 6.2 D3 の鏡像判定は「横移動だけが逆」を見るので、その基準となる
    符号を固定しておく。``motor_ids`` の左右を入れ替えたときに何が変わるかは
    行の入れ替えであって、符号反転では直せない。
    """
    assert m[BACK, 1] == pytest.approx(-1.0)

    ticks = to_ticks(0.0, 0.15, 0.0, m)
    assert ticks[BACK] < 0


# ── ドキュメントに書かれた数値の回帰テスト ──────────────────────────────


def test_reachable_speeds_match_documented_values(m) -> None:
    """base.yaml と README に書かれた到達可能速度が実際に出ること。

    ``max_ticks: 3000`` から逆算される上限は
    x = 0.266 m/s, y = 0.230 m/s, wz = 1.84 rad/s (105 deg/s)。
    ``base_driver`` が起動ログに出す値と同じ式。max_ticks や wheel_radius を
    変えたらドキュメントも直す必要がある、というのをここで検知する。
    """
    wheel_linear_max = math.radians(MAX_TICKS / kin.STEPS_PER_DEG) * WHEEL_RADIUS

    assert wheel_linear_max / abs(m[LEFT, 0]) == pytest.approx(0.266, abs=1e-3)
    assert wheel_linear_max / abs(m[BACK, 1]) == pytest.approx(0.230, abs=1e-3)
    assert wheel_linear_max / BASE_RADIUS == pytest.approx(1.84, abs=1e-2)


@pytest.mark.parametrize(
    "vx, vy, wz_degps, label",
    [
        (0.26, 0.0, 0.0, "max_linear_x"),
        (0.0, 0.23, 0.0, "max_linear_y"),
        (0.0, 0.0, math.degrees(1.8), "max_angular_z"),
    ],
)
def test_config_velocity_limits_do_not_saturate(vx, vy, wz_degps, label, m) -> None:
    """base.yaml の各軸上限を単独で出しても飽和しないこと。

    設定値が到達可能上限を超えていると、指令値と実速度が黙って乖離する
    (base.yaml のコメントが「下の3つはそれ以下に設定する」と言っている理由)。
    3軸を同時に最大にすれば当然飽和するので、ここで見るのは各軸単独。
    """
    ticks = to_ticks(vx, vy, wz_degps, m)
    peak = max(abs(t) for t in ticks)
    assert peak < MAX_TICKS, f"{label} が単独で飽和している (peak={peak})"


# ── 低レベル変換 ────────────────────────────────────────────────────────


def test_steps_per_deg() -> None:
    assert kin.TICKS_PER_REV == 4096.0
    assert kin.STEPS_PER_DEG == pytest.approx(4096.0 / 360.0)


@pytest.mark.parametrize(
    "degps, expected",
    [
        (0.0, 0.0),
        (360.0, 4096),
        (-360.0, -4096),
        (90.0, 1024),
    ],
)
def test_degps_to_ticks(degps, expected) -> None:
    assert kin.degps_to_ticks(degps) == expected


def test_degps_to_ticks_clamps_symmetrically() -> None:
    """飽和は ±32767 で対称。

    sign-magnitude 表現なので -32768 は encode 時に例外になる。lerobot は
    -32768 を許しているが、ここでは意図的に -32767 で止める
    (``kinematics.py`` の docstring 参照)。
    """
    assert kin.degps_to_ticks(1e9) == kin.MAX_TICKS
    assert kin.degps_to_ticks(-1e9) == -kin.MAX_TICKS
    assert kin.MAX_TICKS == 32767


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_ticks_to_radps(sign) -> None:
    """4096 ticks/s = 1 rev/s = 2pi rad/s。"""
    assert kin.ticks_to_radps(4096, sign) == pytest.approx(sign * 2.0 * math.pi)
    assert kin.ticks_to_radps(0, sign) == 0.0


def test_encoder_position_deltas_without_wrap() -> None:
    np.testing.assert_array_equal(
        kin.encoder_position_deltas([110, 190, 320], [100, 200, 300]),
        [10, -10, 20],
    )


def test_encoder_position_deltas_across_wrap() -> None:
    np.testing.assert_array_equal(
        kin.encoder_position_deltas([5, 4090], [4090, 5]),
        [11, -11],
    )


def test_encoder_position_deltas_rejects_shape_mismatch() -> None:
    with pytest.raises(ValueError, match="形が異なる"):
        kin.encoder_position_deltas([1, 2, 3], [1, 2])


# ── 「横移動を指令したのに旋回する」の切り分け ──────────────────────────
#
# 「横移動を指令したのに旋回した」ときに**どのノブを疑えばよいか**を固定する。
#
# 根拠は 1 行で言える: 運動学行列 M の第 3 列は 3 輪とも +base_radius なので、
# **回転成分は 3 輪の速度の平均に比例する**。並べ替えは多重集合を変えず、
# 取付角のずれも 3 輪が 120° 等間隔なので平均を変えない。
# → **平均を変えられるのは wheel_direction_signs だけ**。
#
# ★ 由来 (2026-08-08): 実機で「j / l で旋回する」という報告が出て、ここまで
#   絞り込んだ。**が、原因は運動学ではなかった。** 動かしていたのが
#   `teleop_twist_keyboard` で、あちらは j / l が旋回 (strafe は Shift+J/L)。
#   base.yaml は既定値のままで正しい。
#
#   教訓としてテストは残す。次に本当に「並進を指令したのに旋回する」が
#   起きたとき、**最初に見るべきノブが 1 つに決まる**のは有用なので。
#   ただし**まず動かしているプログラムのキー割り当てを疑うこと**。


def _realized(m, permutation=(0, 1, 2), signs=(1.0, 1.0, 1.0)) -> np.ndarray:
    """モデルと実機の対応付けを取り違えたときに実際に出る車体速度の写像。

    ``v_real = _realized(...) @ v_command``
    """
    p = np.zeros((3, 3))
    for physical, model_row in enumerate(permutation):
        p[physical, model_row] = 1.0
    return np.linalg.inv(m) @ (np.diag(signs) @ p @ m)


@pytest.mark.parametrize("offset", [-180.0, -150.0, -120.0, -60.0, -30.0, 0.0, 90.0])
def test_取付角のずれは並進を旋回にできない(offset) -> None:
    """``wheel_angle_offset_deg`` を間違えても横移動が旋回にはならない。

    3 輪が 120° 等間隔なので、どの角度を足しても 3 輪の速度の平均は変わらない。
    → 実機の「横移動で旋回する」の原因からこのノブを除外できる。
    """
    realized = np.linalg.inv(kin.wheel_matrix(BASE_RADIUS, -90.0)) @ kin.wheel_matrix(
        BASE_RADIUS, offset
    )
    assert (realized @ np.array([0.0, 1.0, 0.0]))[2] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize(
    "permutation",
    [(0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)],
)
def test_motor_idsの並べ替えは並進を旋回にできない(m, permutation) -> None:
    """``motor_ids`` の順序を間違えても横移動が旋回にはならない。

    並べ替えは 3 輪の速度の**多重集合**を変えないので平均も変わらない。
    → 実機の「横移動で旋回する」の原因からこのノブも除外できる。
    """
    assert (_realized(m, permutation=permutation) @ np.array([0.0, 1.0, 0.0]))[
        2
    ] == pytest.approx(0.0, abs=1e-12)


def test_後輪の符号反転だけが前進を保ったまま横移動を旋回にする(m) -> None:
    """「前進は正常・横移動だけ旋回」に合う取り違えは 1 つだけ。

    符号反転 7 通りのうち、**前進が完全に正常のまま**横移動が旋回になるのは
    back(8) の反転だけ。他は前進も壊れる (逆走するか横に流れる)。
    → もし本当にこの症状が出たら ``wheel_direction_signs: [1.0, -1.0, 1.0]``。

    ★ この機体では**まだ一度も該当していない**。base.yaml は既定値のまま
      正しく動いている (2026-08-08 実機確認)。
    """
    forward, sideways = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])

    back = _realized(m, signs=(1.0, -1.0, 1.0))
    assert back @ forward == pytest.approx(forward), "前進が正常でなくなった"
    assert abs((back @ sideways)[2]) > 5.0, "横移動が旋回になっていない"

    others = [s for s in itertools.product([1.0, -1.0], repeat=3)
              if s not in {(1.0, 1.0, 1.0), (1.0, -1.0, 1.0)}]
    for signs in others:
        realized = _realized(m, signs=signs)
        assert realized @ forward != pytest.approx(forward), (
            f"signs={signs} も前進を保ってしまう。原因が一意に絞れない"
        )
