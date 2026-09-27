# SO-101 follower ROS 2

> **★ 通常の運用は [`../robot/`](../robot/) を使ってください（1 イメージ 1 コンテナの統合スタック）。**
> このディレクトリは**このサブシステムだけを切り分けたいとき**のために残してあります。
> 統合スタックと同時に起動しないこと（`ROS_DOMAIN_ID` が衝突して `/tf` が混信します）。

Linuxホスト上のDockerでSO-101フォロワアームを制御します。LeRobot 0.5.1が
実機I/O、モーター設定、較正を担当し、ROS 2 Jazzyが標準の`ros2_control` APIを
提供します。

## システム構成

```text
FollowJointTrajectory / rqt / Cartesian jog
                    ↓
          ros2_control controllers
                    ↓
  JointStateTopicSystem（標準アダプタ）
                    ↓  sensor_msgs/JointState
            LeRobot ROS bridge
                    ↓
              SO101Follower
```

ユーザーからの軌道指令とグリッパ指令は`ros2_control`の各コントローラが
受け取ります。`JointStateTopicSystem`は関節指令と関節状態を
`sensor_msgs/JointState`でLeRobot ROS bridgeと交換します。ブリッジは次を担当します。

- ROS関節名とLeRobotモーター名の対応付け
- 回転関節のrad/degree変換とグリッパ開度の変換
- LeRobot `SO101Follower`への指令送信と状態取得
- 関節速度の推定とROSへの状態配信
- 指令の検証、watchdog、異常時の安全停止

機体固有のモーターID、回転方向、ホーミングオフセット、可動範囲はLeRobotの
較正JSONで管理します。起動時にブリッジが較正JSONとサーボEEPROMの整合性を
確認します。

## 動作環境

- Linuxホスト
- Docker EngineとDocker Compose v2
- USBシリアルデバイスとして接続したSO-101フォロワアーム
- RViz 2を表示する場合はX11またはXWayland

## 安全上の注意

- 実機起動前にアームを低い安定姿勢へ置いてください。
- 正常終了またはwatchdog作動時、ブリッジはLeRobot経由でトルクを切ります。
  アームが脱力して落下するため、停止前にも低い姿勢へ移動してください。
- `SIGKILL`、ホスト停止、電源断ではソフトウェアによるトルクOFFを保証できません。
- 起動時はトルクOFF中に現在位置を`Goal_Position`へラッチしてからLeRobotの
  モーター設定を適用します。
- 較正JSONとサーボEEPROMが一致しない場合、ROS側から自動修復せず起動を中止します。

## 1. モーター設定と較正

LeRobot 0.5.1を用意し、実機を接続して次を実行します。`robot.id`は以後すべての
起動で同じ値を使用してください。

```bash
lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port=/dev/so101_follower

lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/so101_follower \
  --robot.id=my_follower
```

較正ファイルはLeRobot 0.5.1では次に保存されます。

```text
~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json
```

composeはこのディレクトリをコンテナの同じ場所へread-onlyでmountします。
ROS起動中に較正を作成・変更することはできません。

## 2. コンテナの構築

ワークスペースは常にホストから`/ros2_ws`へmountします。初回はイメージを
構築した後、上流ソースの取得とcolcon buildを行います。

```bash
cd docker/so101_ros2
docker compose build
docker compose run --rm so101-follower bash /bootstrap.sh
docker compose up -d
```

`src`、xacro、Pythonコードを変更した場合はイメージを再構築せず、必要に応じて
次のコマンドでホスト側の`build/install`を更新します。

```bash
docker compose run --rm so101-follower bash /bootstrap.sh
docker compose up -d
```

`docker compose down`やイメージの作り直し後も、ソースとビルド成果物はホスト側に
残ります。実行方式を切り替えるための別composeファイルはありません。

実機デバイスを渡す場合は`.env`を作成します。

```dotenv
SO101_DEVICE=/dev/serial/by-id/usb-1a86_…
DIALOUT_GID=20
```

パスはリポジトリ直下の `make serial-ids` で調べます（`usb-1a86_…` が
サーボバス基板）。launch の `usb_port` も同じパスにします。

## 3. 起動

引数なしではmock backendを使用し、シリアルポートを開きません。

```bash
docker compose exec -it so101-follower /entrypoint.sh \
  ros2 launch so101_bringup follower.launch.py
```

実機は明示的に`backend:=lerobot`と較正IDを指定します。

```bash
docker compose exec -it so101-follower /entrypoint.sh \
  ros2 launch so101_bringup follower.launch.py \
    backend:=lerobot \
    usb_port:=/dev/so101_follower \
    robot_id:=my_follower
```

主なlaunch引数は次のとおりです。

| 引数 | 既定値 | 用途 |
| --- | --- | --- |
| `backend` | `mock` | `mock`または`lerobot` |
| `usb_port` | `/dev/so101_follower` | 実機のシリアルポート |
| `robot_id` | 空 | 実機で必須のLeRobot較正ID |
| `calibration_dir` | `/root/.cache/huggingface/lerobot/calibration/robots/so_follower` | read-only較正ディレクトリ |
| `start_rviz` | `true` | RViz 2を起動するか |

実機の`backend:=lerobot`では、launch時に指定した`robot_id`の校正JSONから
`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`の機械的な
`range_min/max`をradへ変換し、同じ`robot_description`の関節limitへ反映します。
`wrist_roll`は全周回転モータとして扱われるため、較正値からは変換せず、
この構成の機械安全範囲 `-3.0〜3.0 rad` を使用します。
校正ファイルが無い場合は実機launchを開始せず、mock backendでは従来のURDF既定値を使います。

較正作業は、ROSを停止した状態でLeRobotのコマンドを使用してください。

## ROSインターフェース

- `/joint_states`
- `/joint_trajectory_controller/joint_trajectory`
- `/joint_trajectory_controller/follow_joint_trajectory`
- `/gripper_controller/gripper_cmd`
- `/so101/hardware_commands`（内部）
- `/so101/hardware_states`（内部）
- `/so101/lerobot_bridge/ready`（起動同期用）

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
ros2 control list_controllers
```

Cartesian jogとキーボード操作の既存launchも、同じJointTrajectoryインターフェースを
利用します。

## watchdogと異常時動作

Controller Managerは内部コマンドを50 Hzで常時発行します。ブリッジは最初の正常な
6関節指令を受け取った後、0.5秒のwatchdogを有効化します。次の場合はトルクOFFを
試み、ブリッジを異常終了させ、launch全体を停止します。

- コマンドが0.5秒以上途絶えた
- LeRobotの読書きで例外が発生した
- 不正な値が続き、正常なコマンドを受信できない

NaN、無限値、不足・重複・未知の関節を含む指令は実機へ転送しません。

## 動作確認

実機を接続する前にmock backendでコンテナ、launch、コントローラの起動を
確認してください。

```bash
docker compose build
docker compose run --rm so101-follower bash /bootstrap.sh
docker compose up -d
docker compose exec -it so101-follower /entrypoint.sh \
  ros2 launch so101_bringup follower.launch.py backend:=mock
```

別の端末から、コントローラと関節状態を確認します。

```bash
docker compose exec -it so101-follower /entrypoint.sh \
  ros2 control list_controllers
docker compose exec -it so101-follower /entrypoint.sh \
  ros2 topic echo /joint_states --once
```
