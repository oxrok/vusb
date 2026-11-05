#!/bin/sh
# The init of the guest tools/qemu-check.sh boots: reports the USB
# devices it sees on the serial console, and tries the nodes they get.
# Every line the check reads starts with "VUSB-GUEST:".
export PATH=/usr/bin:/usr/sbin:/bin:/sbin
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
echo "VUSB-GUEST: up"
modprobe cdc_acm 2>/dev/null
modprobe ftdi_sio 2>/dev/null

# The devices on the bus, root hubs left out: "VID:PID product".
devices() {
    for d in /sys/bus/usb/devices/*; do
        [ -r "$d/idVendor" ] || continue
        vid=$(cat "$d/idVendor")
        [ "$vid" = 1d6b ] && continue
        echo "$vid:$(cat "$d/idProduct") $(cat "$d/product" 2>/dev/null)"
    done
}

# Writes $2 to tty $1 and reads as many bytes back.
echo_test() {
    stty -F "$1" raw -echo 2>/dev/null || return 1
    n=$(printf %s "$2" | wc -c)
    timeout 5 head -c "$n" "$1" >/tmp/echoed &
    reader=$!
    sleep 1
    printf %s "$2" >"$1"
    wait "$reader"
    cat /tmp/echoed
}

mkdir -p /tmp
last=none
while :; do
    now=$(devices)
    if [ "$now" != "$last" ]; then
        last=$now
        if [ -z "$now" ]; then
            echo "VUSB-GUEST: gone"
        else
            echo "VUSB-GUEST: device $now"
            sleep 2
            for h in /sys/class/hidraw/hidraw*; do
                [ -e "$h" ] && echo "VUSB-GUEST: hidraw $(basename "$h")"
            done
            for t in /dev/ttyACM0 /dev/ttyUSB0; do
                [ -e "$t" ] && echo "VUSB-GUEST: echo $(basename "$t") $(echo_test "$t" hello)"
            done
            echo "VUSB-GUEST: checked"
        fi
    fi
    sleep 1
done
