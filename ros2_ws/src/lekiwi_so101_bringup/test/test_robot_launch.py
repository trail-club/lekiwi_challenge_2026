"""robot.launch.py のベース側 3 分岐と start_arm の分岐を検査する。

    sim:=true                            -> sim_nav.launch.py       (dry_run + fake_scan)
    sim:=false かつ use_saved_map:=false -> nav.launch.py           (SLAM)
    sim:=false かつ use_saved_map:=true  -> nav_with_map.launch.py  (AMCL)

★ ここを間違えると「実機のはずが dry_run で走らない」「SLAM と AMCL が同時に
  map->odom を出して TF が二重定義になる」といった、起動して初めて分かる壊れ方をする。
  条件そのもの (AndSubstitution / NotSubstitution) を評価して確かめる。

★ ノードは 1 つも起動しない。LaunchContext に設定値を入れて条件式を評価するだけ。
"""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch", reason="ROS 2 の launch が要る (コンテナ内でのみ実行)")
pytest.importorskip(
    "ament_index_python", reason="robot.launch.py の import に要る (コンテナ内でのみ実行)"
)

from launch import LaunchContext  # noqa: E402
from launch.conditions import IfCondition  # noqa: E402
from launch.substitutions import (  # noqa: E402
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)

# ★ launch ファイルは Python モジュールとして import できない場所にあるので
#   (パッケージではなく share/launch 直下) パス指定で読み込む。
#   起動はしない。_validate_motor_bus_mode を直接呼ぶためだけ。
_LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "robot.launch.py"
_spec = importlib.util.spec_from_file_location("robot_launch", _LAUNCH_PATH)
robot_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(robot_launch)

SIM = LaunchConfiguration("sim")
SAVED = LaunchConfiguration("use_saved_map")
START_BASE = LaunchConfiguration("start_base")
START_CAMERA = LaunchConfiguration("start_camera")
START_ARM = LaunchConfiguration("start_arm")

# robot.launch.py と同じ式。式を変えたらここも変えること。
REAL = NotSubstitution(SIM)
USE_SLAM = AndSubstitution(REAL, NotSubstitution(SAVED))
USE_AMCL = AndSubstitution(REAL, SAVED)
NO_ARM = NotSubstitution(START_ARM)
BASE_OWNS_RSP = NO_ARM
BASE_START_RVIZ = AndSubstitution(LaunchConfiguration("start_rviz"), NO_ARM)
CAMERA = AndSubstitution(AndSubstitution(START_CAMERA, REAL), START_ARM)

BRANCHES = {
    "slam": AndSubstitution(START_BASE, USE_SLAM),
    "amcl": AndSubstitution(START_BASE, USE_AMCL),
    "sim": AndSubstitution(START_BASE, SIM),
}


def _evaluate(condition, **settings):
    context = LaunchContext()
    for name, value in settings.items():
        context.launch_configurations[name] = value
    return IfCondition(condition).evaluate(context)


def _active_branches(**settings):
    return {name for name, cond in BRANCHES.items() if _evaluate(cond, **settings)}


@pytest.mark.parametrize(
    ("sim", "use_saved_map", "expected"),
    [
        ("false", "false", {"slam"}),
        ("false", "true", {"amcl"}),
        ("true", "false", {"sim"}),
        # ★ sim:=true が use_saved_map より強い。実機なし検証で AMCL を上げても
        #   保存地図が無いので map_server が落ちるだけ。
        ("true", "true", {"sim"}),
    ],
)
def test_exactly_one_base_launch_is_selected(sim, use_saved_map, expected):
    active = _active_branches(sim=sim, use_saved_map=use_saved_map, start_base="true")
    assert active == expected, f"sim={sim} use_saved_map={use_saved_map} -> {active}"


@pytest.mark.parametrize("sim", ["true", "false"])
@pytest.mark.parametrize("use_saved_map", ["true", "false"])
def test_start_base_false_disables_every_branch(sim, use_saved_map):
    """start_base:=false ならベース側は 1 つも上がらない。"""
    assert _active_branches(sim=sim, use_saved_map=use_saved_map, start_base="false") == set()


@pytest.mark.parametrize(
    ("sim", "start_camera", "start_arm", "expected"),
    [
        ("false", "true", "true", True),
        ("false", "false", "true", False),
        # ★ sim:=true では start_camera:=true を無視する。RealSense 実機が無い
        #   環境で上げても "No RealSense devices were found" を吐き続けるだけ。
        ("true", "true", "true", False),
        ("true", "false", "true", False),
        # ★ アームが無ければ手首カメラの親 (arm_gripper_link) も無い。
        #   点群は出るが map 上に置けないので起動しない。
        ("false", "true", "false", False),
        ("true", "true", "false", False),
    ],
)
def test_camera_needs_real_hardware_and_an_arm(sim, start_camera, start_arm, expected):
    assert _evaluate(
        CAMERA, sim=sim, start_camera=start_camera, start_arm=start_arm
    ) is expected


@pytest.mark.parametrize(
    ("start_arm", "start_rviz", "rsp", "rviz"),
    [
        # アームを起動するなら RSP と RViz は arm.launch.py が持つ。
        ("true", "true", False, False),
        ("true", "false", False, False),
        # アームが無ければ同じ 2 つをベース側が持つ。RViz は start_rviz に従う。
        ("false", "true", True, True),
        ("false", "false", True, False),
    ],
)
def test_exactly_one_owner_of_rsp_and_rviz(start_arm, start_rviz, rsp, rviz):
    """/robot_description の publisher を 2 つにしないための分岐。

    ★ 2 つあると TRANSIENT_LOCAL / depth 1 の latch をどちらが掴むか非決定になり、
      RViz に別のロボットが出る (CLAUDE.md)。
    """
    settings = {"start_arm": start_arm, "start_rviz": start_rviz}
    assert _evaluate(BASE_OWNS_RSP, **settings) is rsp
    assert _evaluate(BASE_START_RVIZ, **settings) is rviz
    # アーム側は start_arm そのもの。両方 true になる組み合わせが無いことを見る。
    assert _evaluate(START_ARM, **settings) is not rsp


# ─────────────────────────────────────────────────────────────────────────
# start_arm と motor_bus_mode の組み合わせ検査 (_validate_motor_bus_mode)
# ─────────────────────────────────────────────────────────────────────────
def _validate(**settings):
    context = LaunchContext()
    context.launch_configurations.update(
        {"start_arm": "true", "start_base": "true", "motor_bus_mode": ""}
    )
    context.launch_configurations.update(settings)
    return robot_launch._validate_motor_bus_mode(context)


@pytest.mark.parametrize("mode", ["split", "shared"])
def test_arm_requires_an_explicit_bus_mode(mode):
    assert _validate(start_arm="true", motor_bus_mode=mode) == []


@pytest.mark.parametrize("mode", ["", "both", "SPLIT"])
def test_arm_rejects_a_missing_or_unknown_bus_mode(mode):
    with pytest.raises(RuntimeError, match="motor_bus_mode"):
        _validate(start_arm="true", motor_bus_mode=mode)


@pytest.mark.parametrize("mode", ["", "split", "shared"])
def test_armless_machine_may_omit_the_bus_mode(mode):
    """アームが無ければバスの分け方は関係ない。空でも通る。"""
    assert _validate(start_arm="false", motor_bus_mode=mode) == []


def test_armless_machine_still_rejects_an_unknown_bus_mode():
    with pytest.raises(RuntimeError, match="motor_bus_mode"):
        _validate(start_arm="false", motor_bus_mode="both")


def test_starting_neither_arm_nor_base_is_refused():
    """両方 false は「ノードが 1 つも上がらない」。黙って成功させない。"""
    with pytest.raises(RuntimeError, match="start_base"):
        _validate(start_arm="false", start_base="false")
