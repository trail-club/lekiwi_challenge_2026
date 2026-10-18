"""SLAM で構築した地図をファイルに保存する。

nav.launch.py が動いている状態で呼び出す。

    ros2 run lekiwi_base_bringup save_map             # → /maps/my_room
    ros2 run lekiwi_base_bringup save_map living_room # → /maps/living_room
    ros2 run lekiwi_base_bringup save_map /tmp/test   # 絶対パスも可

/maps/ はコンテナ起動時にホストの MAP_DIR をマウントしたディレクトリ。
ホストから確認するには MAP_DIR (既定: ~/maps) を参照。

1 回の呼び出しで 2 種類を保存する:

  * <名前>.posegraph / <名前>.data
      slam_toolbox の /slam_toolbox/serialize_map で書く。
      nav_with_map.launch.py (slam_toolbox の localization モード) が読むのはこちら。
  * <名前>.yaml / <名前>.pgm
      nav.launch.py が (start_slam:=true のとき) 起動する map_saver_server の
      /map_saver/save_map で書く。画像として地図を確認する用。

★ posegraph は SLAM 中の slam_toolbox からしか書き出せない。.pgm から
  posegraph は作れないので、localization で使う地図は必ずこれで保存すること。
"""

from __future__ import annotations

import sys

import rclpy
import rclpy.utilities
from nav2_msgs.srv import SaveMap
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph


def _call(node: Node, srv_type, name: str, req, hint: str):
    """サービスを呼んで応答を返す。見つからない・応答が無いときは None。"""
    client = node.create_client(srv_type, name)
    node.get_logger().info(f"{name} を待機中 (最大 10 秒)...")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"{name} サービスが見つかりません。\n{hint}")
        return None
    future = client.call_async(req)
    # ★ serialize_map は地図の大きさに比例して時間がかかるので 30 秒待つ
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done():
        node.get_logger().error(f"タイムアウト: {name} が 30 秒以内に応答しませんでした")
        return None
    return future.result()


def main(args=None) -> None:
    rclpy.init(args=args)

    # ROS 固有の引数 (--ros-args 以降) を除いてユーザー引数だけ取り出す
    user_args = rclpy.utilities.remove_ros_args(sys.argv[1:])
    name = user_args[0] if user_args else "my_room"

    # 相対名なら /maps/ を付ける、絶対パスならそのまま
    output = name if name.startswith("/") else f"/maps/{name}"

    node = Node("map_saver_client")
    hint = "nav.launch.py (start_slam:=true) が起動しているか確認してください。"
    ok = True

    # ── posegraph (localization 用) ──
    serialize = SerializePoseGraph.Request()
    serialize.filename = output
    node.get_logger().info(f"保存先: {output}.posegraph / {output}.data")
    resp = _call(node, SerializePoseGraph, "/slam_toolbox/serialize_map", serialize,
                 hint + "\n★ nav_with_map.launch.py (localization) 中は保存できません。")
    if resp is None or resp.result != SerializePoseGraph.Response.RESULT_SUCCESS:
        if resp is not None:
            node.get_logger().error(
                f"posegraph の保存に失敗しました (result={resp.result})。"
                "保存先のディレクトリが存在し書き込めるか確認してください。"
            )
        ok = False
    else:
        node.get_logger().info(f"保存完了: {output}.posegraph")

    # ── yaml + pgm (画像) ──
    save = SaveMap.Request()
    save.map_url = output
    save.image_format = "pgm"
    save.map_mode = "trinary"
    save.free_thresh = 0.25
    save.occupied_thresh = 0.65
    node.get_logger().info(f"保存先: {output}.yaml / {output}.pgm")
    resp = _call(node, SaveMap, "/map_saver/save_map", save, hint)
    if resp is None or not resp.result:
        if resp is not None:
            node.get_logger().error("保存に失敗しました (map_saver が false を返しました)")
        ok = False
    else:
        node.get_logger().info(f"保存完了: {output}.yaml")

    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
