#!/bin/sh
# Serves an example device over usbredir, boots a Linux guest in QEMU
# that takes it, and checks what the guest sees: the device, its
# node, an echo through it, an unplug and a replug. No root.
#
#   tools/qemu-check.sh [EXAMPLE]      (default cdc-echo)
#
# With uac-mic the guest also records a second of audio, which needs
# arecord and the alsa configuration on this machine, and snd-usb-audio
# in its kernel. With uvc-cam it captures ten frames, which needs
# v4l2-ctl on this machine and uvcvideo in its kernel.
#
# The guest runs this machine's kernel, or the installed one whose
# release VUSB_GUEST_KERNEL names.
#
# Needs qemu-system-x86_64 with the usb-redir device, dracut, and that
# kernel readable in /boot. Without KVM the guest is emulated
# and takes a minute or two to boot.
set -eu
example=${1:-cdc-echo}
here=$(cd "$(dirname "$0")/.." && pwd)
vusb=$here/target/debug/vusb
ctl=$here/target/debug/vusb-ctl
release=${VUSB_GUEST_KERNEL:-$(uname -r)}
kernel=/boot/vmlinuz-$release
dir=$(mktemp -d /tmp/vusb-qemu.XXXXXX)
console=$dir/console.log
ctl() { "$ctl" -s "$dir/c.sock" "$@"; }

# How many times the guest's console has a line starting with $1.
said() {
    [ -r "$console" ] || { echo 0; return; }
    grep -c "^VUSB-GUEST: $1" "$console" || true
}

# Waits until line $1 has appeared $2 times on the guest's console.
guest_says() {
    tries=0
    until [ "$(said "$1")" -ge "$2" ]; do
        tries=$((tries + 1))
        if [ "$tries" -gt "${3:-600}" ]; then
            echo "the guest never said '$1' ($2 times); console in $console"
            tail -5 "$console" 2>/dev/null || true
            exit 1
        fi
        sleep 0.5
    done
}

finish() {
    [ -n "${qemu:-}" ] && kill "$qemu" 2>/dev/null || true
    [ -n "${server:-}" ] && kill "$server" 2>/dev/null || true
}
trap finish EXIT

dracut --no-hostonly --no-early-microcode --modules base \
    --add-drivers "xhci_pci xhci_hcd usbhid hid_generic cdc_acm ftdi_sio usbserial snd_usb_audio uvcvideo" \
    --install "stty head cat sleep modprobe timeout wc mount mkdir basename printf od tr sed" \
    --install-optional "arecord v4l2-ctl" --include /usr/share/alsa /usr/share/alsa \
    --include "$here/tools/frames.sh" /vusb-frames \
    --include "$here/tools/qemu-guest-init.sh" /vusb-init \
    --force "$dir/initrd.img" "$release" >"$dir/dracut.log" 2>&1

"$vusb" run "$example" --usbredir "$dir/r.sock" --control "$dir/c.sock" \
    --log "$dir/tx.log" </dev/null 2>"$dir/vusb.err" &
server=$!
tries=0
until [ -S "$dir/r.sock" ] && [ -S "$dir/c.sock" ]; do
    tries=$((tries + 1))
    [ "$tries" -gt 100 ] && { cat "$dir/vusb.err"; exit 1; }
    sleep 0.1
done

accel=tcg
[ -w /dev/kvm ] && accel=kvm
qemu-system-x86_64 -machine accel=$accel -m 512 -display none -no-reboot \
    -kernel "$kernel" -initrd "$dir/initrd.img" \
    -append "console=ttyS0 rdinit=/vusb-init panic=-1" \
    -serial "file:$console" \
    -device qemu-xhci \
    -chardev "socket,id=vusb,path=$dir/r.sock" -device usb-redir,chardev=vusb \
    >"$dir/qemu.log" 2>&1 &
qemu=$!

guest_says checked 1
grep '^VUSB-GUEST:' "$console"
ctl status | grep -E '^(transport|guest|configuration)='

echo "--- unplug"
gone=$(said gone)
ctl unplug
guest_says gone $((gone + 1)) 120
echo "--- replug"
ctl replug
guest_says checked 2 240
grep '^VUSB-GUEST:' "$console" | sed -n '/^VUSB-GUEST: gone/,$p'

# The microphone: a second of audio is 96000 bytes, and it starts as the
# model's 1 kHz tone does.
if [ "$example" = uac-mic ]; then
    tone="VUSB-GUEST: audio 96000 00005b0890107e180020f626412dc632"
    [ "$(grep -c "^$tone" "$console")" -ge 2 ] || {
        echo "the guest did not record the tone twice:"
        grep '^VUSB-GUEST: audio' "$console" || true
        exit 1
    }
fi

# The camera: ten frames each time, every one the pattern byte for byte
# (the guest checks that, and prints their numbers), in rising order.
# The guest may lose a frame, so the numbers need not be consecutive.
if [ "$example" = uvc-cam ]; then
    # The console's lines end in a carriage return.
    grep '^VUSB-GUEST: video' "$console" | tr -d '\r' >"$dir/captures" || true
    [ "$(wc -l <"$dir/captures")" -ge 2 ] || { echo "the guest did not capture twice"; exit 1; }
    while read -r _ _ numbers; do
        count=0
        last=-1
        for n in $numbers; do
            case $n in
                *[!0-9]*) echo "not the pattern: $numbers"; exit 1 ;;
            esac
            [ "$n" -gt "$last" ] || { echo "frame $n after frame $last"; exit 1; }
            last=$n
            count=$((count + 1))
        done
        [ "$count" = 10 ] || { echo "$count frames, not ten: $numbers"; exit 1; }
    done <"$dir/captures"
fi

violations=$(ctl status | sed -n 's/^violations=//p')
echo "violations=$violations"
ctl quit
wait "$server" || true
server=
[ "$violations" = 0 ] || { echo "the guest broke the protocol; log in $dir/tx.log"; exit 1; }
echo "qemu check passed; files in $dir"
