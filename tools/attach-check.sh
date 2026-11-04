#!/bin/sh
# Attaches the relay board through vhci-hcd, drives it with vusb-ctl and
# checks the machine sees it come and go. Needs root and vhci-hcd.
set -eu
dir=$(mktemp -d)
socket=$dir/c.sock
ctl="target/debug/vusb-ctl -s $socket"
target/debug/vusb run hid-relay --attach --open-devices --listen 127.0.0.1:0 \
    --control "$socket" --log "$dir/tx.log" </dev/null 2>"$dir/err" &
server=$!
tries=0
until grep -q '^READY' "$dir/err"; do
    tries=$((tries + 1))
    if [ "$tries" -gt 200 ] || ! kill -0 "$server" 2>/dev/null; then
        cat "$dir/err"
        exit 1
    fi
    sleep 0.1
done
lsusb -d 16c0:05df
lsusb -v -d 16c0:05df >/dev/null 2>&1 || true
$ctl arm event=unplug when=in=0:1
lsusb -v -d 16c0:05df >/dev/null 2>&1 || true
$ctl sync
$ctl status | grep -x unplugged=true
if lsusb -d 16c0:05df; then
    echo "still on the bus after the unplug"
    exit 1
fi
$ctl replug
lsusb -d 16c0:05df
$ctl options relays=4
$ctl reset
lsusb -v -d 16c0:05df 2>/dev/null | grep USBRelay4
$ctl quit
wait "$server"
grep -q '^# section replug' "$dir/tx.log"
echo "attach check passed; files in $dir"
