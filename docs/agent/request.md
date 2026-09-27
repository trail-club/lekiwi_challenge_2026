# 依頼（Mac → 実機）

- **更新**: 2026-09-27
- **状態**: 実行待ち
- **安全区分**: 🟡（**B は CP2102 の EEPROM への永続的な書き込み。人間がその場にいて許可したときだけ**。D は車輪を浮かせる）
- **ブランチ**: `feat/udev-by-id`
- **対象機体**: アームを取り外した LeKiwi と、RPLIDAR A1 **2 台**（どちらもシリアルが `0001`）

## 何を確認してほしいか

udev を「全機体共通のルール 1 つ」に変え、機体は `.env` の `*_DEVICE` に
`/dev/serial/by-id/` のパスで書く方式にしました。コンテナ内の名前
（`/dev/lekiwi` `/dev/rplidar`）は変わりません。

RPLIDAR の CP2102 は全個体のシリアルが `0001` で by-id では区別できないので、
個体ごとに書き換えます。Mac では compose の展開と Makefile の分岐までしか
確かめられていません。**実機で初めて分かるのは次の 4 つ**です。

1. `cp210x-cfg` がビルドでき、シリアルを書き換えられるか
2. 書き換えたあと by-id の名前が変わり、2 台が別の名前になるか
3. by-id の実際の名前（`.env.example` の例は典型的な形で、実測ではない）
4. by-id のパスを bind したコンテナで、ベースと LiDAR が動くか

## 手順

`sudo` が使える PC で行うこと（共用機 dgx-spark ではなく）。

```bash
git fetch && git switch feat/udev-by-id && git pull
```

### A. 現状を記録する（読むだけ）

```bash
ls -l /dev/serial/by-id/
ls -l /etc/udev/rules.d/ | grep -E 'lekiwi|so101|rplidar|robot'
```

### B. RPLIDAR のシリアルを書き換える（🟡 永続的な書き込み。人間の許可が要る）

```bash
sudo apt install libusb-1.0-0-dev
git clone https://github.com/DiUS/cp210x-cfg.git ~/cp210x-cfg
make -C ~/cp210x-cfg
```

★ **LiDAR を 1 台だけ挿した状態で**、1 台ずつ行う。`-d` を必ず付ける。

```bash
sudo ~/cp210x-cfg/cp210x-cfg -l                        # bus:dev を控える
sudo ~/cp210x-cfg/cp210x-cfg -d <bus:dev>              # 書く前の値（全部貼る）
sudo ~/cp210x-cfg/cp210x-cfg -d <bus:dev> -S rplidar-01
# USB を挿し直す
ls -l /dev/serial/by-id/
```

2 台目は `rplidar-02` で同じことをする。**書いた値は LiDAR 本体にテープで貼って控える。**
`-S` 以外のオプション（`-V` `-P` `-N`）は使わないこと。

### C. udev ルールと .env

```bash
make udev-dry-run
make install-udev
make serial-ids                 # 2 台の LiDAR を両方挿した状態で
cd docker/robot
# .env の LEKIWI_DEVICE と RPLIDAR_DEVICE に by-id パスを書く。SO101_DEVICE は空
```

### D. 起動（🟡 車輪を浮かせる）

```bash
make run-base START_RVIZ=false  # 前面で走らせる（Ctrl+C で止める）
```

別端末で:

```bash
cd docker/robot
make check-base
```

## 確認項目

| # | 見るもの | 期待 |
| --- | --- | --- |
| B-1 | `cp210x-cfg` のビルド | 通る |
| B-2 | 書く前の `-d <bus:dev>` の出力 | serial が `0001`（全部貼る） |
| B-3 | 書いて挿し直したあとの `ls -l /dev/serial/by-id/` | 名前の末尾が `rplidar-01` / `rplidar-02` になる |
| B-4 | 2 台同時に挿した `make serial-ids` | LiDAR が **2 本の別の名前**で見える |
| C-1 | `make install-udev` の出力 | 以前の `99-lekiwi.rules` 等が残っていれば NOTE が出る（消さない） |
| C-2 | `udevadm info -q property -n <by-id パス> \| grep ID_MM` | `ID_MM_DEVICE_IGNORE=1` |
| C-3 | `.env` を空にして `make up-base` | 理由付きのエラーで止まり、コンテナは上がらない |
| D-1 | `make up-base` | コンテナが上がる |
| D-2 | コンテナ内で `ls -l /dev/lekiwi /dev/rplidar` | 両方とも文字デバイス |
| D-3 | `ros2 topic hz /scan` | 出る |
| D-4 | 車輪（**浮かせたまま**） | `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'` で回る |
| D-5 | `Ctrl+C` のあと `make release BUS_MODE=base` | ホイールだけ解放される |

## ★ 気をつけること

- **B は取り消せない書き込み。** 元に戻すには `-S 0001` でもう一度書くしかない
- `docker kill` を使わないこと。非常停止は**物理スイッチ**だけ
- 以前の `/etc/udev/rules.d/99-lekiwi.rules` 等は**消さない**（共用機で他の人が使っている可能性がある）

## 報告してほしいこと

`docs/agent/report.md` に B-1〜D-5 の結果を書いてください。
**by-id の名前は省略せずそのまま**貼ってください（`.env.example` の例を実測値に直すため）。
失敗したものはログの該当行も。
