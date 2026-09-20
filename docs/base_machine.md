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

## 4. 起動（SLAM）

```bash
make run-base
```

`Ctrl+C` で止める。**止める前に端末を閉じないこと**（SIGKILL では停止処理が
走らず、ホイールが最後の指令速度で回り続ける）。

X が無い端末では RViz が落ちるので付ける:

```bash
make run-base START_RVIZ=false
```

別端末での確認:

```bash
make check-base
```

期待値: `/robot_description` の publisher = 1、`/joint_states` = 1（車輪 3 関節）、
アームのノードが 0、`/navigate_to_pose` が見える、`map → base_footprint` と
`base_link → laser_link` が引ける。

---

## 5. 走らせる

**★ 動作確認は車輪を浮かせてから。**

```bash
# コンテナの中（別端末で make shell）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
```

`/cmd_vel` は 0.5 秒で失効して停止する。ゴールを与えるなら RViz の
"2D Goal Pose"、または:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0}, orientation: {w: 1.0}}}'
```

---

## 6. 地図を保存する

`make run-base` を**走らせたまま別端末で**。

```bash
make save-map                   # → /maps/my_room.yaml と .pgm
make save-map MAP_NAME=living   # → /maps/living.yaml
```

| | パス |
| --- | --- |
| コンテナ内 | `/maps/<MAP_NAME>.yaml` と `.pgm` |
| ホスト | `$MAP_DIR/<MAP_NAME>.yaml`（既定 `~/maps`） |

★ 保存できるのは `make run-base`（SLAM）の間だけ。`run-base-map` にはこの
サービスが無い。

---

## 7. 保存地図で走る（AMCL）

```bash
make run-base-map MAP_NAME=my_room
make run-base-map MAP_FILE=/maps/other.yaml   # パスを直接指定する場合
```

起動後、**RViz の "2D Pose Estimate" で初期姿勢を与える**。

★ AMCL は `update_min_d: 0.25` m / `update_min_a: 0.2` rad 動くまで更新しない。
**止まっている間は `map → odom` が 1 ミリも動かないのが正常。** ズレたまま
走らせても直らない（`recovery_alpha_*` が 0 で大域再測位は無効）ので、
大きく外れたら 2D Pose Estimate を打ち直す。

---

## 8. 停止

```bash
# run-base の端末で Ctrl+C（ホイールの速度ゼロ + トルク OFF）
make down       # コンテナを片付ける
```

**★ 非常停止は物理スイッチだけ。** `docker kill` を使わないこと
（SIGKILL では停止処理が走らず、ホイールが回り続ける）。

---

## 9. 異常終了したとき

launch が落ちた / SIGKILL された / ホイールが走り出した場合。

```bash
make release-check BUS_MODE=base    # 読むだけ。いまトルクが入っているか
make release BUS_MODE=base          # ホイールを止めてトルクを切る
```

**コンテナは落とさなくてよい。** 止まっている必要があるのは launch だけ。
`BUS_MODE=base` はホイール（`/dev/lekiwi` の ID 7/8/9）しか触らない。

---

## コマンド一覧

| コマンド | 内容 |
| --- | --- |
| `make install-udev BUS_MODE=base` | `/dev/lekiwi` と `/dev/rplidar` の udev ルール |
| `make build` / `make bootstrap` | イメージとワークスペース |
| `make up-base` | コンテナだけ起動 |
| `make run-base` | 起動（SLAM） |
| `make run-base-map` | 起動（保存地図 + AMCL） |
| `make mock-base` | 実機なし（シリアルも LiDAR も開かない） |
| `make shell` | コンテナに入る |
| `make check-base` | ROS グラフの確認 |
| `make save-map` | 地図の保存 |
| `make release BUS_MODE=base` | 異常終了からの復帰 |
| `make down` | コンテナの停止・削除 |

`make run-base` の実体:

```bash
ros2 launch lekiwi_base_bringup nav.launch.py \
    port:=/dev/lekiwi serial_port:=/dev/rplidar
```

`port` と `serial_port` の既定は `/dev/lekiwi` と `/dev/rplidar` なので、
コンテナの中で手で叩くなら引数は要らない。`make` が明示的に渡しているのは
`.env` の `LEKIWI_DEVICE` / `RPLIDAR_DEVICE` を変えた機体に追従するため。
