# 依頼（Mac → 実機）

- **更新**: 2026-09-20
- **状態**: 未着手
- **安全区分**: 🟡（**車輪を必ず浮かせること**。アームは載っていない前提）
- **ブランチ**: `feat/armless-lekiwi`
- **対象機体**: ★ **アームを取り外した LeKiwi**（`/dev/so101_follower` は存在しない）

## 何を確認してほしいか

`robot.launch.py` に `start_arm:=false` を足しました。アームを外した機体で
ベース・LiDAR・SLAM・Nav2 だけが上がることを確認してください。

Mac では mock（`sim:=true`）までしか検証できていません。**実機で初めて分かるのは
「シリアルを開いて車輪が回るか」と「LiDAR が出るか」の 2 つ**です。

## 手順

```bash
git pull
cd docker/robot

# 1. .env に LEKIWI_SERIAL を設定（SO101_SERIAL は空でよい）
make udev-dry-run BUS_MODE=base     # 生成されるルールを見るだけ
make install-udev BUS_MODE=base
ls -l /dev/lekiwi /dev/rplidar      # ★ /dev/so101_follower は無くてよい

make bootstrap                      # 初回のみ

# 2. ★ 車輪を浮かせる。ここから車輪が回りうる
make run-base                       # 前面で走らせる（Ctrl+C で止める）
```

別端末で:

```bash
cd docker/robot
make check-base
```

## 確認項目

| # | 見るもの | 期待 |
| --- | --- | --- |
| A-1 | `make up-base` でコンテナが上がる | `/dev/so101_follower` を mount しないので起動する |
| A-2 | `make check-base` の `/robot_description` | publisher = **1** |
| A-3 | `make check-base` の `/joint_states` | publisher = **1**、名前は車輪 3 関節のみ |
| A-4 | `make check-base` のアームのノード | **1 つも居ない**（`so101*` / `controller_manager`） |
| A-5 | `ros2 topic hz /scan` | RPLIDAR が実際に回って出ている |
| A-6 | **車輪が回るか** | `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'`（★ 浮かせたまま） |
| A-7 | `make check-base` の TF | `map → base_footprint` と `base_link → laser_link` が引ける |
| A-8 | RViz | ベースだけのモデルが出る。アームのリンクは無い |
| A-9 | `Ctrl+C` | 車輪が止まり、トレースバック無しで終わる |
| A-10 | **`make release BUS_MODE=base`** | launch を止めてから実行。ホイールだけ解放し、アームの ID 1〜6 は探しに行かない |
| A-11 | `make release-check BUS_MODE=base` | 読むだけ。A-10 の前後で `Torque_Enable` が変わる |

## ★ 気をつけること

- **`docker kill` を使わないこと。** 車輪が最後の指令速度で回り続けます。
  非常停止は**物理スイッチ**だけです
- A-6 は**必ず車輪を浮かせてから**。`/cmd_vel` は 0.5 秒で失効しますが、
  機体が動き出す可能性があります
- `make run-split` / `make run-shared` はこの機体では使えません
  （`/dev/so101_follower` が無いのでコンテナが上がりません）

## 報告してほしいこと

`docs/agent/report.md` に、A-1〜A-11 の結果を書いてください。
**動いたかどうかだけでなく、実際に出た値**（publisher 数、TF の並進、`/scan` の Hz）を
そのまま貼ってください。失敗したものはログの該当行も。
