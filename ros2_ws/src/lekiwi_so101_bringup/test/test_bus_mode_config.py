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


def test_launch_port_defaults_match_what_docker_mounts():
    """launch の既定ポート名と docker が bind mount する名前を揃える。

    ★ ずれていると sllidar_node や base_driver **だけ**が黙って死ぬ。
      launch 全体は上がるので気付きにくい (/scan が出ず、slam_toolbox が
      map->odom を出さず、Nav2 が Invalid frame ID map を INFO で吐き続ける)。
    ★ /dev/ttyUSB0 のような番号付きの名前を既定にしないこと。挿し直すと変わる。
    """
    compose = (ROOT / "docker/robot/compose.yaml").read_text()
    assert "${RPLIDAR_DEVICE:-/dev/rplidar}" in compose
    for overlay in ("compose.split.yaml", "compose.shared.yaml", "compose.base.yaml"):
        text = (ROOT / "docker/robot" / overlay).read_text()
        assert "${LEKIWI_DEVICE:-/dev/lekiwi}" in text, overlay

    for name in ("nav.launch.py", "nav_with_map.launch.py"):
        launch = (
            ROOT / "ros2_ws/src/lekiwi_base_bringup/launch" / name
        ).read_text()
        assert 'DeclareLaunchArgument("port", default_value="/dev/lekiwi"' in launch, name
        assert (
            'DeclareLaunchArgument("serial_port", default_value="/dev/rplidar"' in launch
        ), name


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
