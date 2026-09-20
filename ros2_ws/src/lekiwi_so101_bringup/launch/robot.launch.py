"""ロボット全体を 1 プロセスで起動する launch。

    # 実機 (docker/robot のコンテナ内で)
    ros2 launch lekiwi_so101_bringup robot.launch.py \
      motor_bus_mode:=split backend:=lerobot robot_id:=my_follower

    # 実機なし (Mac でも動く)
    ros2 launch lekiwi_so101_bringup robot.launch.py motor_bus_mode:=split sim:=true

    # アームを取り外した機体 (ベース + LiDAR だけ)
    ros2 launch lekiwi_so101_bringup robot.launch.py start_arm:=false

    # 保存済み地図 + AMCL
    ros2 launch lekiwi_so101_bringup robot.launch.py motor_bus_mode:=split \\
      backend:=lerobot robot_id:=my_follower use_saved_map:=true \\
      map_file:=/maps/my_room.yaml

────────────────────────────────────────────────────────────────────────
起動するもの
────────────────────────────────────────────────────────────────────────
    robot_state_publisher (結合 URDF。システム全体でこれ 1 つ)  ┐ arm.launch.py
    LeRobot ブリッジ + ros2_control + spawner                   │ を include
    RViz (システム全体でこれ 1 つ)                              ┘

    base_driver + scan_filter                                   ┐ nav.launch.py /
    sllidar_node (sim:=true なら fake_scan)                     │ sim_nav.launch.py /
    slam_toolbox または map_server+amcl / Nav2 / map_saver      ┘ nav_with_map.launch.py

    realsense2_camera (手首カメラ)                                d435i.launch.py

★ start_arm:=false — アームを取り外した機体
  アームを物理的に外した LeKiwi ではこれを渡す。arm.launch.py を include せず、
  ベース側の include に robot_state_publisher と RViz を持たせる。

    * URDF は結合 URDF ではなく lekiwi_description の **ベース単体 URDF**。
      アームのリンクも ros2_control も最初から存在しない
    * motor_bus_mode は**渡さなくてよい**。アームが無ければ
      ベースは常に /dev/lekiwi の ID 7/8/9 を自分で開く (hardware_backend:=serial)。
      shared を渡しても bridge にはならない (橋渡しするアームが居ないため)
    * 手首カメラは arm_gripper_link に付くので**起動しない**。
      start_camera:=true を渡しても無視してログに理由を出す
    * 較正 JSON もアームのトルクも関係しないので、停止は Ctrl+C だけでよい。
      異常終了からの復帰は `make release-wheels BUS_MODE=split`

★ arm.launch.py との違い
  arm.launch.py は**アーム側だけ** (RSP + ブリッジ + ros2_control + RViz)。
  この launch がそれを include したうえでベース・LiDAR・カメラも起動する。
  アームだけを切り分けて確認したいときは arm.launch.py を直接使える。

★ リーチと逆運動学は起動しない。lekiwi_examples へ移した。
  この launch はロボットを「動かせる状態」にするところまでで、その上で
  何をするかはアプリケーション側の責任にする。別ターミナルで:

      ros2 launch lekiwi_examples reach.launch.py      # map 上の点へリーチ
      ros2 run lekiwi_examples teleop_keyboard         # キーボード操作

★ 停止
  この launch を Ctrl+C する。アームのトルクが切れて落ちるので、
  lekiwi_examples の reach.launch.py も起動している場合は、先に
  `ros2 service call /so101/stow std_srvs/srv/Trigger {}` で畳んでおくこと。
  SIGKILL された場合の復帰は `ros2 run lekiwi_so101_bringup release_all`。

★ include を GroupAction で包む理由
  launch の設定値は既定で共有スコープに入る。included な launch が同名の引数
  (start_rviz, rviz_config, serial_port ...) を宣言すると、その既定値が
  こちらの設定を**上書きする**。GroupAction (scoped=True) で閉じ込める。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PythonExpression,
)

MOUNT_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def _is_true(context, name):
    """launch の条件式と同じ真偽判定。IfCondition が受け付ける値だけを真とする。"""
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def _validate_motor_bus_mode(context):
    """起動前に設定の矛盾を止める。

    ★ アームを起動するなら motor_bus_mode は必須のまま。既定を置かないと
      launch 自身が「Required launch argument」で落ち、アームの無い機体で
      意味のない値を強制することになるので、既定は空文字にしてここで弾く。
    ★ アームを起動しないなら motor_bus_mode は**使わない**。
      ベースは常に自分で /dev/lekiwi を開く (下の hardware_backend を参照)。
    """
    mode = LaunchConfiguration("motor_bus_mode").perform(context)
    start_arm = _is_true(context, "start_arm")

    if start_arm and mode not in ("split", "shared"):
        raise RuntimeError("motor_bus_mode must be explicitly set to split or shared")
    if not start_arm and mode not in ("", "split", "shared"):
        raise RuntimeError(
            f"motor_bus_mode に {mode!r} は使えない (split / shared / 未指定)"
        )
    # ★ 両方 false は「何も起動しない」。黙って上がると原因を探すことになる。
    if not start_arm and not _is_true(context, "start_base"):
        raise RuntimeError(
            "start_arm:=false と start_base:=false の両方は指定できない "
            "(起動するノードが 1 つも無くなる)"
        )
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("lekiwi_so101_bringup"))
    base_launch_dir = Path(get_package_share_directory("lekiwi_base_bringup")) / "launch"
    camera_launch_dir = Path(get_package_share_directory("realsense_bringup")) / "launch"

    sim = LaunchConfiguration("sim")
    use_saved_map = LaunchConfiguration("use_saved_map")
    map_file = LaunchConfiguration("map_file")
    lekiwi_port = LaunchConfiguration("lekiwi_port")
    lidar_port = LaunchConfiguration("lidar_port")
    start_lidar = LaunchConfiguration("start_lidar")
    start_base = LaunchConfiguration("start_base")
    start_camera = LaunchConfiguration("start_camera")
    start_rviz = LaunchConfiguration("start_rviz")
    start_arm = LaunchConfiguration("start_arm")

    backend = LaunchConfiguration("backend")
    motor_bus_mode = LaunchConfiguration("motor_bus_mode")
    robot_id = LaunchConfiguration("robot_id")
    usb_port = LaunchConfiguration("usb_port")
    split_calibration_dir = LaunchConfiguration("split_calibration_dir")
    shared_calibration_dir = LaunchConfiguration("shared_calibration_dir")
    joint_prefix = LaunchConfiguration("joint_prefix")

    arm_port = PythonExpression([
        "'", lekiwi_port, "' if '", motor_bus_mode,
        "' == 'shared' else '", usb_port, "'",
    ])
    calibration_dir = PythonExpression([
        "'", shared_calibration_dir, "' if '", motor_bus_mode,
        "' == 'shared' else '", split_calibration_dir, "'",
    ])
    # ★ bridge はアームの LeRobot ブリッジが共有バスを所有しているときだけ。
    #   アームを起動しないならその橋が無いので、shared 機でもベースが自分で
    #   /dev/lekiwi を開いて ID 7/8/9 を回す (base.yaml の motor_ids はそのまま)。
    hardware_backend = PythonExpression([
        "'bridge' if '", motor_bus_mode, "' == 'shared' and '",
        start_arm, "'.lower() in ('true', '1') else 'serial'",
    ])

    # ★ robot_state_publisher と RViz は**システム全体で 1 つずつ**。
    #   アームを起動するなら arm.launch.py が結合 URDF で持ち、
    #   起動しないならベース側の include がベース単体 URDF で持つ。
    #   /robot_description は TRANSIENT_LOCAL / depth 1 なので、publisher が
    #   2 つあると RViz に別のロボットが出る (CLAUDE.md)。
    no_arm = NotSubstitution(start_arm)
    base_owns_rsp = no_arm
    base_start_rviz = AndSubstitution(start_rviz, no_arm)

    wrist_camera = LaunchConfiguration("wrist_camera")
    wrist_camera_name = LaunchConfiguration("wrist_camera_name")
    wrist_camera_parent = LaunchConfiguration("wrist_camera_parent")
    wrist_camera_fps = LaunchConfiguration("wrist_camera_fps")
    wrist_camera_decimation = LaunchConfiguration("wrist_camera_decimation")
    wrist_camera_reconnect_timeout = LaunchConfiguration(
        "wrist_camera_reconnect_timeout"
    )
    mock_wrist_camera_optical = LaunchConfiguration("mock_wrist_camera_optical")

    # ベース側 3 通りの排他条件。
    #   sim:=true                          -> sim_nav.launch.py   (dry_run + fake_scan)
    #   sim:=false かつ use_saved_map:=false -> nav.launch.py       (SLAM)
    #   sim:=false かつ use_saved_map:=true  -> nav_with_map.launch.py (AMCL)
    real = NotSubstitution(sim)
    use_slam = AndSubstitution(real, NotSubstitution(use_saved_map))
    use_amcl = AndSubstitution(real, use_saved_map)

    return LaunchDescription([
        # ───────── 全体 ─────────
        DeclareLaunchArgument(
            "sim", default_value="false",
            description="true: シリアルも LiDAR も開かない (base_driver は dry_run、"
                        "スキャンは fake_scan、アームは backend:=mock を推奨)"),
        DeclareLaunchArgument("start_base", default_value="true",
                              description="false にするとベースとナビを起動しない"),
        # ★ アームを取り外した機体で false にする。arm.launch.py を include せず、
        #   robot_state_publisher と RViz はベース側の include が持つ。
        #   URDF もベース単体のものになり、アームのリンクは最初から存在しない。
        DeclareLaunchArgument(
            "start_arm", default_value="true",
            description="false: アームを取り外した機体。arm.launch.py を include せず、"
                        "RSP と RViz をベース側に持たせる。手首カメラも起動しない"),
        DeclareLaunchArgument("start_camera", default_value="true",
                              description="realsense2_camera を起動するか。"
                                          "★ sim:=true と start_arm:=false では"
                                          "自動的に起動しない"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        # ★ 既定が空文字なのは「アームを起動しないなら指定不要」にするため。
        #   アームを起動するなら _validate_motor_bus_mode が空を弾く。
        DeclareLaunchArgument(
            "motor_bus_mode", default_value="",
            description="REQUIRED (start_arm:=true のとき): split (two ports) or "
                        "shared (one canonical ID 1-9 bus)。"
                        "start_arm:=false では使われない",
        ),
        OpaqueFunction(function=_validate_motor_bus_mode),

        # ───────── ベース ─────────
        DeclareLaunchArgument("lekiwi_port", default_value="/dev/lekiwi"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument(
            "start_lidar", default_value="true",
            description="実機の sllidar_node を起動するか。sim:=true では無視される"),
        DeclareLaunchArgument(
            "use_saved_map", default_value="false",
            description="true: slam_toolbox の代わりに map_server + AMCL を使う"),
        DeclareLaunchArgument(
            "map_file", default_value="",
            description="use_saved_map:=true のとき必須。例 /maps/my_room.yaml"),

        # ───────── アーム ─────────
        DeclareLaunchArgument(
            "backend", default_value="mock",
            description="mock: シリアルを開かない / lerobot: 実機の SO-101"),
        DeclareLaunchArgument(
            "robot_id", default_value="",
            description="backend:=lerobot では必須の LeRobot 較正 ID"),
        DeclareLaunchArgument("usb_port", default_value="/dev/so101_follower"),
        DeclareLaunchArgument(
            "arm_torque", default_value="true",
            description="false: アームのトルクを入れず指令も書かない。"
                        "手で動かして /joint_states を読むとき"),
        DeclareLaunchArgument(
            "split_calibration_dir",
            default_value="/root/.cache/huggingface/lerobot/calibration/robots/so_follower"),
        DeclareLaunchArgument(
            "shared_calibration_dir",
            default_value="/root/.cache/huggingface/lerobot/calibration/robots/lekiwi"),
        DeclareLaunchArgument("joint_prefix", default_value="arm_"),

        # ───────── 手首カメラ ─────────
        # ★ WRIST_CAMERA_* は「URDF にリンクを生やすか」と「その取付姿勢」。
        #   既定は空文字 = 指定なしで、URDF 側の既定値が唯一の情報源になる。
        DeclareLaunchArgument(
            "wrist_camera", default_value=os.environ.get("WRIST_CAMERA", "true")),
        DeclareLaunchArgument(
            "wrist_camera_name",
            default_value=os.environ.get("WRIST_CAMERA_NAME", "wrist_camera"),
            description="★ realsense の camera_name と URDF の両方に効く。"
                        "この launch は同じ値を両方へ渡すので不一致は起きない"),
        DeclareLaunchArgument(
            "wrist_camera_parent",
            default_value=os.environ.get("WRIST_CAMERA_PARENT", "gripper_link"),
            description="接頭辞なしで書く (joint_prefix が前置される)"),
        *(DeclareLaunchArgument(
            f"wrist_camera_{key}",
            default_value=os.environ.get(f"WRIST_CAMERA_{key.upper()}", ""),
            description="空なら URDF の既定値を使う")
          for key in MOUNT_KEYS),
        DeclareLaunchArgument(
            "wrist_camera_fps", default_value=os.environ.get("WRIST_CAMERA_FPS", "6")),
        DeclareLaunchArgument(
            "wrist_camera_decimation",
            default_value=os.environ.get("WRIST_CAMERA_DECIMATION", "2")),
        DeclareLaunchArgument(
            "wrist_camera_reconnect_timeout",
            default_value=os.environ.get(
                "WRIST_CAMERA_RECONNECT_TIMEOUT", "6.0"
            ),
            description="RealSense切断後の再接続試行間隔 [s]",
        ),
        # ★ 実機では realsense2_camera がこの TF を出すので起動しないこと。
        #   sim:=true でカメラ実機が無いときだけ true にする。
        DeclareLaunchArgument("mock_wrist_camera_optical", default_value="false"),

        # ───────── アーム + RSP + リーチ + RViz ─────────
        # ★ start_arm:=true のとき、arm.launch.py がシステム全体で唯一の
        #   robot_state_publisher と RViz を持つ。だからベース側の include には
        #   start_robot_state_publisher:=false / start_rviz:=false が渡る。
        #   start_arm:=false ではこの GroupAction ごと消え、同じ 2 つを
        #   ベース側が持つ (base_owns_rsp / base_start_rviz)。
        GroupAction(condition=IfCondition(start_arm), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "arm.launch.py")),
                launch_arguments=[
                    ("backend", backend),
                    ("motor_bus_mode", motor_bus_mode),
                    ("robot_id", robot_id),
                    ("usb_port", arm_port),
                    # robot.launch.py 側は arm_torque。ベースにもトルクがあるので
                    # どちらの話か分かる名前にしてある。下位は torque。
                    ("torque", LaunchConfiguration("arm_torque")),
                    ("calibration_dir", calibration_dir),
                    ("joint_prefix", joint_prefix),
                    ("start_rviz", start_rviz),
                    ("wrist_camera", wrist_camera),
                    ("wrist_camera_name", wrist_camera_name),
                    ("wrist_camera_parent", wrist_camera_parent),
                    ("mock_wrist_camera_optical", mock_wrist_camera_optical),
                    *((f"wrist_camera_{key}", LaunchConfiguration(f"wrist_camera_{key}"))
                      for key in MOUNT_KEYS),
                ],
            ),
        ]),

        # ───────── ベース: 実機 + SLAM ─────────
        GroupAction(
            condition=IfCondition(AndSubstitution(start_base, use_slam)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(base_launch_dir / "nav.launch.py")),
                    launch_arguments=[
                        ("port", lekiwi_port),
                        ("hardware_backend", hardware_backend),
                        ("serial_port", lidar_port),
                        ("start_lidar", start_lidar),
                        ("start_robot_state_publisher", base_owns_rsp),
                        ("start_rviz", base_start_rviz),
                    ],
                ),
            ],
        ),

        # ───────── ベース: 実機 + 保存済み地図 + AMCL ─────────
        GroupAction(
            condition=IfCondition(AndSubstitution(start_base, use_amcl)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(base_launch_dir / "nav_with_map.launch.py")),
                    launch_arguments=[
                        ("port", lekiwi_port),
                        ("hardware_backend", hardware_backend),
                        ("serial_port", lidar_port),
                        ("map_file", map_file),
                        ("start_lidar", start_lidar),
                        ("start_robot_state_publisher", base_owns_rsp),
                        ("start_rviz", base_start_rviz),
                    ],
                ),
            ],
        ),

        # ───────── ベース: 実機なし ─────────
        # ★ base_driver は dry_run 固定、スキャンは fake_scan。
        #   シリアルも LiDAR も一切開かないので Mac でも動く。
        GroupAction(
            condition=IfCondition(AndSubstitution(start_base, sim)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(base_launch_dir / "sim_nav.launch.py")),
                    launch_arguments=[
                        ("hardware_backend", hardware_backend),
                        ("start_robot_state_publisher", base_owns_rsp),
                        ("start_rviz", base_start_rviz),
                    ],
                ),
            ],
        ),

        # ───────── 手首カメラ ─────────
        # ★ sim:=true では起動しない。USB カメラが無い環境で
        #   realsense2_camera を上げても "No RealSense devices were found" を
        #   吐き続けるだけで、模擬光学フレームは mock_wrist_camera_optical が担う。
        # ★ start_arm:=false でも起動しない。手首カメラは arm_gripper_link に
        #   付くので、アームが無ければ TF ツリーに繋がる先が無い。点群は出るが
        #   map 上に置けず、RViz では "Fixed Frame への変換が無い" になるだけ。
        GroupAction(
            condition=IfCondition(
                AndSubstitution(
                    AndSubstitution(start_camera, NotSubstitution(sim)), start_arm)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(camera_launch_dir / "d435i.launch.py")),
                    launch_arguments=[
                        # ★ camera_name は TF のフレーム名にも効く。
                        #   結合 URDF の wrist_camera_name と同じ値をここから渡すので、
                        #   4 コンテナ構成で起きていた「compose の env と URDF の
                        #   不一致」は原理的に起きない。
                        ("camera_name", wrist_camera_name),
                        ("camera_namespace", wrist_camera_name),
                        ("enable_pointcloud", "true"),
                        ("depth_fps", wrist_camera_fps),
                        ("color_fps", wrist_camera_fps),
                        ("decimation", wrist_camera_decimation),
                        ("reconnect_timeout", wrist_camera_reconnect_timeout),
                        ("start_rviz", "false"),
                    ],
                ),
            ],
        ),

        # ★ sim:=true では start_camera:=true を無視する。黙って無視すると
        #   「カメラを上げたつもり」になるので、ログに理由を出す。
        LogInfo(
            condition=IfCondition(AndSubstitution(start_camera, sim)),
            msg="sim:=true なので realsense2_camera は起動しません。"
                "光学フレームだけ要るなら mock_wrist_camera_optical:=true。",
        ),
        LogInfo(
            condition=IfCondition(
                AndSubstitution(
                    AndSubstitution(start_camera, NotSubstitution(sim)), no_arm)),
            msg="start_arm:=false なので realsense2_camera は起動しません。"
                "手首カメラは arm_gripper_link に付くため、アームの無い URDF では"
                "点群を map 上に置けません。",
        ),
        LogInfo(
            condition=IfCondition(no_arm),
            msg="start_arm:=false: ベース単体 URDF で起動します "
                "(アームのリンク・ros2_control・手首カメラは存在しません)。"
                "robot_state_publisher と RViz はベース側が持ちます。",
        ),
    ])
