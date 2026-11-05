#!/bin/sh
# Serves an example device over usbredir, boots a Linux guest in QEMU
# that takes it, and checks what the guest sees: the device, its
# node, an echo through it, an unplug and a replug. No root.
#
#   tools/qemu-check.sh [EXAMPLE]      (default cdc-echo)
#
# With uac-mic the guest also records a second of audio, which needs
# arecord and the alsa configuration on this machine, and snd-usb-audio
# in its kernel.
#
# Needs qemu-system-x86_64 with the usb-redir device, dracut, and this
# machine's kernel readable in /boot. Without KVM the guest is emulated
# and takes a minute or two to boot.
set -eu
example=${1:-cdc-echo}
here=$(cd "$(dirname "$0")/.." && pwd)
vusb=$here/target/debug/vusb
ctl=$here/target/debug/vusb-ctl
kernel=/boot/vmlinuz-$(uname -r)
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
    --add-drivers "xhci_pci xhci_hcd usbhid hid_generic cdc_acm ftdi_sio usbserial snd_usb_audio" \
    --install "stty head cat sleep modprobe timeout wc mount mkdir basename printf od tr" \
    --install-optional "arecord" --include /usr/share/alsa /usr/share/alsa \
    --include "$here/tools/qemu-guest-init.sh" /vusb-init \
    --force "$dir/initrd.img" "$(uname -r)" >"$dir/dracut.log" 2>&1

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

violations=$(ctl status | sed -n 's/^violations=//p')
echo "violations=$violations"
ctl quit
wait "$server" || true
server=
[ "$violations" = 0 ] || { echo "the guest broke the protocol; log in $dir/tx.log"; exit 1; }
echo "qemu check passed; files in $dir"
