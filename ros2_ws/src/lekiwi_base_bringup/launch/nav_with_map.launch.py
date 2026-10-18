"""保存済みの地図を使って slam_toolbox 自己位置推定 + Nav2 ナビゲーションを動かす launch。

    ros2 launch lekiwi_base_bringup nav_with_map.launch.py map_file:=/maps/my_room

■ 使い方の流れ

  1. nav.launch.py (SLAM モード) で地図を作る
  2. 地図を保存する:
       ros2 run lekiwi_base_bringup save_map my_room
       → /maps/my_room.posegraph と /maps/my_room.data (この launch が読む)
         /maps/my_room.yaml と /maps/my_room.pgm (画像として見る用) が生成される
  3. nav.launch.py を停止する
  4. 地図を作り始めた位置と向きに機体を置き、この launch を起動する:
       ros2 launch lekiwi_base_bringup nav_with_map.launch.py map_file:=/maps/my_room
  5. 別の位置から始めたときは RViz の "2D Pose Estimate" で自己位置を与える
  6. RViz の "Nav2 Goal" でゴールを与えるとナビゲーションが始まる

■ nav.launch.py との違い
  * slam_toolbox を localization モード (localization_slam_toolbox_node) で動かす。
    保存した posegraph を読み込み、スキャンを既存の地図に突き合わせて map → odom を出す
  * 地図は保存されない。map_saver も起動しない (SLAM モードに戻すには nav.launch.py)

■ map_file に渡すもの
  ★ slam_toolbox が読むのは .posegraph / .data で、.yaml / .pgm ではない。
    .pgm からは posegraph を作り直せないので、地図は save_map で保存し直すこと。
  拡張子は付けても付けなくてもよい。以下はどれも /maps/my_room.posegraph を読む:
    map_file:=/maps/my_room
    map_file:=/maps/my_room.yaml
    map_file:=/maps/my_room.posegraph

■ 初期自己位置
  起動時は initial_pose_x / initial_pose_y / initial_pose_yaw (既定 0, 0, 0) を
  初期位置として突き合わせる。map 座標の原点は地図を作り始めた位置なので、
  そこに置いて起動すれば指定は要らない。
  ずれていたら RViz の "2D Pose Estimate" (/initialpose) で与え直す。
  slam_toolbox はその付近でスキャンマッチをやり直す。

■ /map について
  ★ localization モードの slam_toolbox は、保存した地図に直近の数スキャン
    (scan_buffer_size 個) を重ねた /map を map_update_interval ごとに出し直す。
    直近のスキャンは使い終わると捨てるので、保存した地図ファイルは変わらない。

■ start_lidar フラグ (nav.launch.py と共通)
  * start_lidar:=true  (既定): このプロセス内で sllidar_node を起動する
  * start_lidar:=false       : /scan を外部 (rplidar コンテナ等) に任せる
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition

# map_file に付いていたら外す拡張子。slam_toolbox は拡張子なしの名前を受け取り、
# 自分で .posegraph と .data を付けて読む。
MAP_SUFFIXES = (".posegraph", ".data", ".yaml", ".pgm")


def posegraph_base(map_file: str) -> str:
    """map_file から slam_toolbox の map_file_name (拡張子なし) を作る。"""
    for suffix in MAP_SUFFIXES:
        if map_file.endswith(suffix):
            return map_file[: -len(suffix)]
    return map_file


def _slam_localization(context):
    map_file = LaunchConfiguration("map_file").perform(context)
    base = posegraph_base(map_file)
    # ★ ファイルが無いとき、slam_toolbox は "Failed to read file" をログに出すだけで
    #   起動を続け、map → odom を出さないまま Nav2 が TF 待ちで止まる。
    #   起動前に弾く。
    if not Path(base + ".posegraph").is_file() or not Path(base + ".data").is_file():
        raise RuntimeError(
            f"{base}.posegraph / {base}.data がありません (map_file:={map_file})。"
            "nav.launch.py の実行中に `ros2 run lekiwi_base_bringup save_map <名前>` "
            "で保存した地図を指定してください。.yaml / .pgm だけでは読み込めません。"
        )

    start_pose = [
        float(LaunchConfiguration(name).perform(context))
        for name in ("initial_pose_x", "initial_pose_y", "initial_pose_yaw")
    ]

    # ★ slam_toolbox は Jazzy では LifecycleNode。configure → activate を
    #   発行しないと unconfigured のまま止まり、map → odom の TF が出ない。
    #   イベントの組み方は slam_toolbox 2.8.5 の localization_launch.py と同じ。
    #   その launch を include しないのは、パラメータファイルを 1 つしか
    #   受け取らず map_file_name / map_start_pose を上書きできないため。
    node = LifecycleNode(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            # スキャンマッチの調整値は SLAM モードと共通にする (同じ機体・同じ部屋)
            LaunchConfiguration("slam_params_file"),
            {
                "mode": "localization",
                "map_file_name": base,
                # ★ 空のままだと slam_toolbox は地図を読まずに起動する
                #   ("Map starting pose not specified")。
                "map_start_pose": start_pose,
                "use_lifecycle_manager": False,
                "use_sim_time": False,
            },
        ],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        start_state="configuring",
        goal_state="inactive",
        entities=[
            LogInfo(msg=f"[nav_with_map] slam_toolbox を activate する ({base})"),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))
    return [node, configure, activate]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("lekiwi_base_bringup"))
    description_share = Path(get_package_share_directory("lekiwi_description"))
    nav2_bringup_share = Path(get_package_share_directory("nav2_bringup"))

    xacro_file = description_share / "urdf" / "lekiwi_base.urdf.xacro"

    port = LaunchConfiguration("port")
    hardware_backend = LaunchConfiguration("hardware_backend")
    serial_port = LaunchConfiguration("serial_port")
    start_rviz = LaunchConfiguration("start_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    start_lidar = LaunchConfiguration("start_lidar")
    base_params = LaunchConfiguration("base_params_file")
    nav2_params = LaunchConfiguration("nav2_params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = ParameterValue(
        Command(["xacro ", str(xacro_file), " use_mesh:=false"]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/lekiwi",
                              description="LeKiwiベースのシリアルポート"),
        DeclareLaunchArgument("hardware_backend", default_value="serial"),
        # ★ 既定は udev が作る安定名。port (/dev/lekiwi) と揃えてある。
        #   /dev/ttyUSB0 は挿し直すと番号が変わるうえ、docker/robot が
        #   コンテナへ bind mount するのは RPLIDAR_DEVICE (既定 /dev/rplidar)
        #   だけなので、既定が ttyUSB0 だと sllidar_node だけが黙って死ぬ
        #   (/scan が出ず、slam_toolbox が map->odom を出さない)。
        #   udev ルールを入れていない環境では明示すること:
        #     ros2 launch ... serial_port:=/dev/ttyUSB0
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLIDAR A1 のシリアルポート "
                                          "(start_lidar:=true 時のみ使用)。"
                                          "udev で作る安定名"),
        DeclareLaunchArgument(
            "map_file",
            description="save_map で保存した地図。例 /maps/my_room "
                        "(.posegraph / .yaml を付けてもよい)。"
                        "読むのは <名前>.posegraph と <名前>.data。",
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0",
                              description="起動時の自己位置 x [m] (map 座標)"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0",
                              description="起動時の自己位置 y [m] (map 座標)"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0",
                              description="起動時の自己位置 yaw [rad] (map 座標)"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        # /robot_description は TRANSIENT_LOCAL / depth 1 なので、publisher が
        # 2 つあると後から繋いだ購読者がどちらの latch を掴むか非決定になる
        # (CLAUDE.md の「RViz に別のロボットが出る」症状)。アームを載せた
        # 合成 launch は自分でロボット全体の RSP を持つので false にする。
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument(
            "start_lidar", default_value="true",
            description="true: このプロセス内で sllidar_node を起動する。"
                        "false: /scan を外部 (rplidar コンテナ等) に任せる。",
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=str(bringup_share / "config" / "base.yaml"),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=str(bringup_share / "config" / "slam_toolbox.yaml"),
            description="nav.launch.py と共通。mode / map_file_name / "
                        "map_start_pose はこの launch が上書きする",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(bringup_share / "config" / "nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(nav2_bringup_share / "rviz" / "nav2_default_view.rviz"),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(start_robot_state_publisher),
        ),

        Node(
            package="lekiwi_base_bringup",
            executable="base_driver",
            name="lekiwi_base_driver",
            output="screen",
            parameters=[
                base_params,
                {"port": port, "hardware_backend": hardware_backend},
            ],
        ),

        # /scan を機体が映らない角度だけに絞って /scan_filtered として配信する。
        # 地図もこのトピックから作っているので、突き合わせる相手が揃う。
        Node(
            package="lekiwi_base_bringup",
            executable="scan_filter",
            name="scan_angular_filter",
            output="screen",
        ),

        # nav.launch.py と同じ: frame_id は laser_link 固定
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": 115200,
                "frame_id": "laser_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }],
            condition=IfCondition(start_lidar),
        ),

        # slam_toolbox (localization) → TF (map → odom) と /map
        OpaqueFunction(function=_slam_localization),

        # nav.launch.py と同じ。/map は slam_toolbox が出すので map_server は要らない。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_bringup_share / "launch" / "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "false",
                "autostart": "true",
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(start_rviz),
        ),
    ])
