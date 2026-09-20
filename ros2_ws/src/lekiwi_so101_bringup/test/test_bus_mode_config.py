from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


def test_compose_motor_mounts_are_mode_specific():
    common = (ROOT / "docker/robot/compose.yaml").read_text()
    split = (ROOT / "docker/robot/compose.split.yaml").read_text()
    shared = (ROOT / "docker/robot/compose.shared.yaml").read_text()
    assert "SO101_DEVICE" not in common and "LEKIWI_DEVICE" not in common
    assert "SO101_DEVICE" in split and "LEKIWI_DEVICE" in split
    assert "SO101_DEVICE" not in shared and shared.count("LEKIWI_DEVICE") == 2


def test_makefile_has_only_explicit_run_commands():
    makefile = (ROOT / "docker/robot/Makefile").read_text()
    assert "\nrun:" not in makefile
    for target in ("run-split:", "run-shared:", "mock-split:", "mock-shared:"):
        assert target in makefile
    assert "require-bus-mode" in makefile


def _recipe(makefile: str, target: str) -> str:
    """Makefile から 1 つのターゲットのレシピ本文だけを取り出す。"""
    body = makefile.split(f"\n{target}:", 1)[1]
    lines = []
    for line in body.splitlines()[1:]:
        if line and not line.startswith("\t"):
            break
        lines.append(line)
    return "\n".join(lines)


def test_make_targets_only_enter_a_shell():
    """make は launch を起動しない。コンテナを上げてシェルへ入るだけ。

    ★ ここが崩れると **make を叩いた瞬間にトルクが入る**。
      launch はシェルの中で人が叩く (引数が起動ごとに変わるため)。
    """
    makefile = (ROOT / "docker/robot/Makefile").read_text()
    for target in ("run-split", "run-shared", "run-base",
                   "mock-split", "mock-shared", "mock-base"):
        recipe = _recipe(makefile, target)
        assert "$(call enter_shell," in recipe, target
        assert "ros2 launch" not in recipe, target
    # シェルへ入る動作そのものは enter_shell が 1 か所で持つ。
    enter_shell = makefile.split("define enter_shell", 1)[1].split("endef", 1)[0]
    assert "exec -it $(2) bash" in enter_shell


def test_combined_launch_requires_mode_and_selects_bridge_backend():
    launch = (
        ROOT
        / "ros2_ws/src/lekiwi_so101_bringup/launch/robot.launch.py"
    ).read_text()
    assert 'DeclareLaunchArgument(\n            "motor_bus_mode",' in launch
    assert "motor_bus_mode must be explicitly set" in launch
    assert "'bridge' if '" in launch
    assert '("hardware_backend", hardware_backend)' in launch


def test_internal_message_has_fixed_three_tick_order():
    message = (
        ROOT
        / "ros2_ws/src/lekiwi_hardware_interfaces/msg/WheelCommand.msg"
    ).read_text()
    assert message.splitlines() == ["std_msgs/Header header", "int32[3] ticks"]
