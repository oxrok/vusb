#!/bin/sh
# Puts devices on dummy_hcd, a USB device controller in software wired
# to a host controller of the same machine, through a gadget, and
# checks what this machine, the host at the other end, sees: the CDC
# echo and an echo through its tty, an unplug and a replug; then a disk
# that is a file, written and read back block for block.
#
#   sudo tools/gadget-check.sh
#
# Needs root and the dummy_hcd and raw_gadget modules, which not every
# distribution builds: Arch has both, Fedora and Ubuntu one each.
# VUSB and VUSB_CTL name the programs when they are not in target/debug.
set -eu
vusb=${VUSB:-target/debug/vusb}
vusb_ctl=${VUSB_CTL:-target/debug/vusb-ctl}
modprobe dummy_hcd
modprobe raw_gadget
dir=$(mktemp -d)
socket=$dir/c.sock
ctl() { "$vusb_ctl" -s "$socket" "$@"; }

# Waits until $1, a command, succeeds; $2 says what was waited for.
until_ok() {
    tries=0
    until eval "$1"; do
        tries=$((tries + 1))
        if [ "$tries" -gt 150 ]; then
            echo "never happened: $2"
            cat "$dir/err"
            exit 1
        fi
        sleep 0.2
    done
}

# Serves example $1 with options $2 and waits until the host has
# configured it $3 times.
serve() {
    # shellcheck disable=SC2086 # the options are words
    "$vusb" run "$1" $2 --gadget auto --control "$socket" --log "$dir/$1.log" \
        </dev/null 2>"$dir/err" &
    server=$!
    ready 1
}
ready() { until_ok "[ \"\$(grep -c '^READY' \"$dir/err\")\" -ge $1 ]" "the host configuring the device"; }

# The node of class $1 (tty, block) of the USB device of vendor $2.
node() {
    for n in /sys/class/"$1"/*; do
        d=$(readlink -f "$n/device") || continue
        while [ "$d" != / ]; do
            if [ -r "$d/idVendor" ]; then
                [ "$(cat "$d/idVendor")" = "$2" ] && { basename "$n"; return 0; }
                break
            fi
            d=$(dirname "$d")
        done
    done
    return 1
}

echo "--- a serial port"
serve cdc-echo ""
grep -x 1209 /sys/bus/usb/devices/*/idVendor >/dev/null
ctl status | grep -x transport=gadget
ctl status | grep -x host=true
until_ok "node tty 1209 >/dev/null" "the tty of the echo"
tty=/dev/$(node tty 1209)
stty -F "$tty" raw -echo
timeout 10 head -c 5 "$tty" >"$dir/echoed" &
reader=$!
sleep 1
printf hello >"$tty"
wait "$reader"
grep -x hello "$dir/echoed"

echo "--- unplug"
ctl unplug
until_ok "! node tty 1209 >/dev/null" "the tty leaving"
ctl status | grep -x host=false
echo "--- replug"
ctl replug
ready 2
until_ok "node tty 1209 >/dev/null" "the tty coming back"
ctl status | grep -x violations=0
ctl quit
wait "$server"
until_ok "! node tty 1209 >/dev/null" "the tty leaving at the end"

echo "--- a disk"
truncate -s 8M "$dir/disk.img"
serve msc-disk "-o file=$dir/disk.img"
until_ok "node block 1209 >/dev/null" "the disk"
disk=/dev/$(node block 1209)
[ "$(cat "/sys/class/block/$(basename "$disk")/size")" = 16384 ]
# Three megabytes of something, written through USB a megabyte in.
head -c 3145728 /dev/urandom >"$dir/data"
dd if="$dir/data" of="$disk" bs=64k seek=16 oflag=direct status=none
sync
dd if="$disk" bs=64k skip=16 count=48 iflag=direct status=none | cmp - "$dir/data"
# And the file is the disk.
dd if="$dir/disk.img" bs=64k skip=16 count=48 status=none | cmp - "$dir/data"
ctl status | grep -x violations=0
ctl quit
wait "$server"
echo "gadget check passed; files in $dir"
