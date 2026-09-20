"""robot.launch.py のベース側 3 分岐が排他になっていることを検査する。

    sim:=true                            -> sim_nav.launch.py       (dry_run + fake_scan)
    sim:=false かつ use_saved_map:=false -> nav.launch.py           (SLAM)
    sim:=false かつ use_saved_map:=true  -> nav_with_map.launch.py  (AMCL)

★ ここを間違えると「実機のはずが dry_run で走らない」「SLAM と AMCL が同時に
  map->odom を出して TF が二重定義になる」といった、起動して初めて分かる壊れ方をする。
  条件そのもの (AndSubstitution / NotSubstitution) を評価して確かめる。

★ ノードは 1 つも起動しない。LaunchContext に設定値を入れて条件式を評価するだけ。
"""

import pytest

pytest.importorskip("launch", reason="ROS 2 の launch が要る (コンテナ内でのみ実行)")

from launch import LaunchContext  # noqa: E402
from launch.conditions import IfCondition  # noqa: E402
from launch.substitutions import (  # noqa: E402
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)

SIM = LaunchConfiguration("sim")
SAVED = LaunchConfiguration("use_saved_map")
START_BASE = LaunchConfiguration("start_base")
START_CAMERA = LaunchConfiguration("start_camera")

# robot.launch.py と同じ式。式を変えたらここも変えること。
REAL = NotSubstitution(SIM)
USE_SLAM = AndSubstitution(REAL, NotSubstitution(SAVED))
USE_AMCL = AndSubstitution(REAL, SAVED)

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
    ("sim", "start_camera", "expected"),
    [
        ("false", "true", True),
        ("false", "false", False),
        # ★ sim:=true では start_camera:=true を無視する。RealSense 実機が無い
        #   環境で上げても "No RealSense devices were found" を吐き続けるだけ。
        ("true", "true", False),
        ("true", "false", False),
    ],
)
def test_camera_only_starts_on_real_hardware(sim, start_camera, expected):
    condition = AndSubstitution(START_CAMERA, NotSubstitution(SIM))
    assert _evaluate(condition, sim=sim, start_camera=start_camera) is expected
