# アーム無し専用機（LeKiwi ベース）

SO-101 アームを載せない機体の手順。**ベース + RPLIDAR + SLAM + Nav2 だけ**を動かす。

- 起動に使うのは `lekiwi_base_bringup`。`robot.launch.py` は使わない
- Docker は `docker/robot`（1 イメージ 1 コンテナ）を `BUS_MODE=base` で使う
- アーム有りの手順は [`../README.md`](../README.md) と
  [`../docker/robot/README.md`](../docker/robot/README.md)

必要なもの: `/dev/lekiwi`（ホイール 3 輪）、`/dev/rplidar`（RPLIDAR A1）。

---

## 1. `.env`

```bash
cd docker/robot
cp .env.example .env
```

書き換えるのは 2 つだけ。

```bash
DIALOUT_GID=20        # getent group dialout の GID
LEKIWI_SERIAL=        # 下で調べる。SO101_SERIAL は空のままでよい
```

```bash
for d in /dev/ttyACM*; do
  echo "$d $(udevadm info -q property -n "$d" | grep ID_SERIAL_SHORT)"
done
```

---

## 2. udev

```bash
make udev-dry-run BUS_MODE=base     # 生成されるルールを見るだけ
make install-udev BUS_MODE=base     # sudo で /etc/udev/rules.d/ へ入れる
ls -l /dev/lekiwi /dev/rplidar      # ★ 両方できていること
```

`base` は `lekiwi` と `rplidar` の 2 つだけ作る（`so101` は作らない）。

---

## 3. ビルド

```bash
make build
make bootstrap        # 初回とパッケージ追加時
```

---

## 4. コンテナに入る

```bash
make up-base          # コンテナを起動する（ロボットはまだ動かない）
make shell            # コンテナに入る
```

以降のコマンドは**コンテナの中**で叩く。別端末が要るときは、もう一度
`make shell` すればよい。

---

## 5. 起動（SLAM）

```bash
ros2 launch lekiwi_base_bringup nav.launch.py
```

`.env` でデバイス名を変えた場合だけ渡す。

```bash
ros2 launch lekiwi_base_bringup nav.launch.py \
    port:=/dev/lekiwi serial_port:=/dev/rplidar
```

止めるのは `Ctrl+C`。**止める前にシェルを `exit` したり端末を閉じたりしないこと**
（SIGKILL では停止処理が走らず、ホイールが最後の指令速度で回り続ける）。

起動できたかの確認（ホスト側の別端末で）:

```bash
make check-base
```

期待値: `/robot_description` の publisher = 1、`/joint_states` = 1（車輪 3 関節）、
アームのノードが 0、`/navigate_to_pose` が見える、`map → base_footprint` と
`base_link → laser_link` が引ける。

---

## 6. 走らせる

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
```

`/cmd_vel` は 0.5 秒で失効して停止する。ゴールを与えるなら RViz の
"2D Goal Pose"、または:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0}, orientation: {w: 1.0}}}'
```

---

## 7. 地図を保存する

`nav.launch.py` を**走らせたまま別のシェルで**。

```bash
ros2 run lekiwi_base_bringup save_map            # → /maps/my_room.yaml と .pgm
ros2 run lekiwi_base_bringup save_map living     # → /maps/living.yaml
ros2 run lekiwi_base_bringup save_map /tmp/test  # / で始めれば絶対パス
```

| | パス |
| --- | --- |
| コンテナ内 | `/maps/<名前>.yaml` と `.pgm` |
| ホスト | `$MAP_DIR/<名前>.yaml`（既定 `~/maps`） |

★ 保存できるのは `nav.launch.py`（SLAM）の間だけ。`/map_saver/save_map` を出して
いるのはこの launch が起動する `map_saver_server` で、`nav_with_map.launch.py`
（AMCL）側には無い。

---

## 8. 保存地図で走る（AMCL）

```bash
ros2 launch lekiwi_base_bringup nav_with_map.launch.py map_file:=/maps/my_room.yaml
```

`map_file` は**必須引数**。忘れると launch が即座に拒否する。

起動後、**RViz の "2D Pose Estimate" で初期姿勢を与える**。

★ AMCL は `update_min_d: 0.25` m / `update_min_a: 0.2` rad 動くまで更新しない。
**止まっている間は `map → odom` が 1 ミリも動かないのが正常。** ズレたまま
走らせても直らない（`recovery_alpha_*` が 0 で大域再測位は無効）ので、
大きく外れたら 2D Pose Estimate を打ち直す。

止まったまま補正させたいときは、強制更新を叩く（1 回で 1 更新）。

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty {}
```

★ 連打しないこと。同じスキャンを別々の観測として扱うので、粒子が潰れて
**間違った姿勢に自信を持つ**。数回で直らなければ走らせるか打ち直す。

---

## 9. 停止

```bash
# launch を叩いたシェルで Ctrl+C（ホイールの速度ゼロ + トルク OFF）
exit            # シェルを抜ける
make down       # コンテナを片付ける
```

**★ 非常停止は物理スイッチだけ。** `docker kill` を使わないこと
（SIGKILL では停止処理が走らず、ホイールが回り続ける）。

---

## 10. 異常終了したとき

launch が落ちた / SIGKILL された / ホイールが走り出した場合。**ホスト側で**:

```bash
make release-check BUS_MODE=base    # 読むだけ。いまトルクが入っているか
make release BUS_MODE=base          # ホイールを止めてトルクを切る
```

**コンテナは落とさなくてよい。** 止まっている必要があるのは launch だけ。
`BUS_MODE=base` はホイール（`/dev/lekiwi` の ID 7/8/9）しか触らない。

---

## 11. 動作確認でやってほしいこと

以下の topic や機能の確認をしてみてください。

### Hardware Interfaceなど

| 対象 | 確認方法 |
| --- | --- |
| `/cmd_vel` | `ros2 topic pub` を叩く |
| `/scan` | RViz で表示 |
| `/scan_filtered` | RViz で表示 |
| TF `odom → base_footprint` | RViz で表示 |

### SLAM

| 対象 | 確認方法 |
| --- | --- |
| `/map` | RViz で表示 |
| TF `map → odom` | RViz で表示 |
| マップの保存 | `ros2 run lekiwi_base_bringup save_map` |
| Spotting | 適当な場所にロボットを動かし、`ros2 run tf2_ros tf2_echo` で `base_link` の座標を記録する |

### Localization

| 対象 | 確認方法 |
| --- | --- |
| マップの読み込み | `nav_with_map.launch.py` のオプション `map_file` を指定 |
| TF `map → odom` | RViz で表示 |
| `/initialpose` | RViz の "2D Pose Estimate" で初期位置を指定 |
| `/request_nomotion_update` | `ros2 service call` を叩く |

### Navigation

| 対象 | 確認方法 |
| --- | --- |
| `/goal_pose` | RViz の "2D Goal Pose" |
| Spotting した位置へ Navigation | Spotting で記録した位置を用いて `ros2 action send_goal /navigate_to_pose` |
| `/plan` | RViz で表示（Global Planner の下の Path） |
| `/global_costmap/costmap` | RViz で表示（Global Planner の下の Global Costmap） |
| `/local_costmap/costmap` | RViz で表示（Controller の下の Local Costmap） |

---

## よくある症状

| ログ | 原因 |
| --- | --- |
| `ID 7/8/9: 応答なし` → `起動失敗: 応答しないモータ` | **サーボに電気が来ていない。** ポートは開けている。バッテリーのスイッチ・残量・12V の配線・デイジーチェーンを見る |
| `Invalid frame ID "odom"` が `[INFO]` で延々出る | `base_driver` が居ない。★ INFO なのでエラーに見えないが、これは上の一次故障の結果 |
| `Message Filter dropping message: frame 'laser_link'` | 同上。`odom` が無くスキャンを変換できない |
| `rviz2: could not connect to display` | `DISPLAY` が空。`start_rviz:=false` で切るか、X のある端末から `make up-base` し直す（コンテナの `DISPLAY` は作成時に固定される） |
| `/scan` が出ない | `sllidar_node` だけが死んでいる。`serial_port` と `/dev/rplidar` を確認 |

---

## コマンド一覧

ホスト側（`docker/robot` で）:

| コマンド | 内容 |
| --- | --- |
| `make install-udev BUS_MODE=base` | `/dev/lekiwi` と `/dev/rplidar` の udev ルール |
| `make build` / `make bootstrap` | イメージとワークスペース |
| `make up-base` | コンテナを起動 |
| `make shell` | コンテナに入る |
| `make check-base` | ROS グラフの確認 |
| `make release BUS_MODE=base` | 異常終了からの復帰 |
| `make down` | コンテナの停止・削除 |

コンテナの中:

| コマンド | 内容 |
| --- | --- |
| `ros2 launch lekiwi_base_bringup nav.launch.py` | 起動（SLAM） |
| `ros2 launch lekiwi_base_bringup nav_with_map.launch.py map_file:=...` | 起動（保存地図 + AMCL） |
| `ros2 run lekiwi_base_bringup save_map [名前]` | 地図の保存 |
| `ros2 service call /request_nomotion_update std_srvs/srv/Empty {}` | AMCL を静止したまま 1 回更新 |
