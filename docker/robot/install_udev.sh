#!/usr/bin/env bash
# 全機体共通の udev ルール (99-robot-serial.rules) をホストへ入れる。
# PC ごとに 1 回だけでよい。機体を増やしても入れ直す必要は無い。
#
# 機体の区別はこのルールでは行わない。/dev/serial/by-id/ のパスを
# docker/robot/.env の LEKIWI_DEVICE / SO101_DEVICE / RPLIDAR_DEVICE に書く。
set -euo pipefail

usage() {
  echo "usage: $0 [--dry-run]" >&2
  exit 2
}

dry_run=false
case "${1:-}" in
  --dry-run) dry_run=true; shift ;;
  "") ;;
  *) usage ;;
esac
[[ $# -eq 0 ]] || usage

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rule="$script_dir/99-robot-serial.rules"
dest=/etc/udev/rules.d/99-robot-serial.rules

# 以前の方式 (機体ごとのシリアルを埋めて /dev/lekiwi 等を作る) のルール。
# 共用機では他の利用者がまだ使っているかもしれないので、消さずに知らせるだけ。
report_legacy() {
  local found=()
  for f in 99-lekiwi.rules 99-so101.rules 99-rplidar.rules; do
    [[ -e "/etc/udev/rules.d/$f" ]] && found+=("/etc/udev/rules.d/$f")
  done
  if [[ ${#found[@]} -gt 0 ]]; then
    echo "NOTE: 以前の方式のルールが残っています: ${found[*]}"
    echo "      /dev/lekiwi 等はそれが作るもので、この機体を指すとは限りません。"
    echo "      .env の *_DEVICE には /dev/serial/by-id/ のパスを書いてください。"
  fi
}

# 共用機 (dgx-spark など) には root 所有のヘルパーがあり、sudo が制限された
# 一般ユーザーもこれだけは実行できる。ルールの中身はヘルパーが自分で持つ。
# 利用者が書き換えられるこのリポジトリのファイルを root で読ませないため
# (udev ルールは RUN+= で任意のコマンドを走らせる)。
#   install-robot-udev [--dry-run]
# ★ 共用機に入るのはヘルパーのルールで、99-robot-serial.rules ではない。
#   このファイルを変えたらヘルパー側 (trail-club/directory-access) も揃えること。
helper=/usr/local/sbin/install-robot-udev
if [[ -x "$helper" ]]; then
  if [[ "$dry_run" == true ]]; then
    "$helper" --dry-run
    exit 0
  fi
  sudo "$helper"
  report_legacy
  exit 0
fi

if [[ "$dry_run" == true ]]; then
  echo "--- $dest ---"
  sed -n '/^SUBSYSTEM/,/^$/p' "$rule"
  exit 0
fi

sudo install -m 0644 "$rule" "$dest"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo "udevルールをインストールしました: $dest"
report_legacy
echo "次: ls -l /dev/serial/by-id/ で機体のパスを調べ、.env の *_DEVICE に書く"
