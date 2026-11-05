#!/bin/sh
# The init of the guest tools/qemu-check.sh boots: reports the USB
# devices it sees on the serial console, and tries the nodes they get.
# Every line the check reads starts with "VUSB-GUEST:".
export PATH=/usr/bin:/usr/sbin:/bin:/sbin
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
echo "VUSB-GUEST: up"
# Some distributions have loading snd_pcm pull in the sequencer too,
# which this guest does not carry: load it without that rule.
modprobe --ignore-install snd_pcm 2>/dev/null
# Built into some kernels, modules of others.
for m in xhci_pci usbhid hid_generic cdc_acm ftdi_sio snd_usb_audio; do
    modprobe "$m" 2>/dev/null
done

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
last=
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
            # A sound card: one second of what the device hears, its
            # size and its first sixteen bytes.
            if [ -e /proc/asound/card0 ]; then
                arecord -q -D hw:0,0 -f S16_LE -r 48000 -c 1 -d 1 -t raw /tmp/rec.raw 2>/tmp/arecord.err
                echo "VUSB-GUEST: audio $(wc -c </tmp/rec.raw) $(od -An -tx1 -N 16 /tmp/rec.raw | tr -d ' \n')"
            fi
            echo "VUSB-GUEST: checked"
        fi
    fi
    sleep 1
done
