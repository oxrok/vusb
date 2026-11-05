#!/bin/sh
# Attaches the test pattern camera through vhci-hcd and captures from it
# with the kernel's uvcvideo: a stream the host negotiates, checked
# frame by frame. Needs root, vhci-hcd, uvcvideo and v4l2-ctl.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
dir=$(mktemp -d)
socket=$dir/c.sock
ctl="target/debug/vusb-ctl -s $socket"
target/debug/vusb run uvc-cam --attach --listen 127.0.0.1:0 \
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
# The camera's capture node: uvcvideo makes a second one for metadata.
node() {
    for v in /sys/class/video4linux/video*; do
        [ -r "$v/name" ] || continue
        case $(cat "$v/name") in
            *"Test pattern camera"*) [ "$(cat "$v/index")" = 0 ] && basename "$v" ;;
        esac
    done | head -1
}
tries=0
until video=$(node) && [ -n "$video" ]; do
    tries=$((tries + 1))
    [ "$tries" -gt 100 ] && { echo "no video device for the camera"; exit 1; }
    sleep 0.1
done
v4l2-ctl -d "/dev/$video" --get-fmt-video | sed -n 's/^\t\(Width\|Pixel\)/\1/p'
v4l2-ctl -d "/dev/$video" --stream-mmap --stream-count=20 --stream-to="$dir/capture.raw" \
    >/dev/null 2>&1
# Twenty frames, each the pattern byte for byte. The host takes them as
# fast as it can, so it skips some: their numbers only have to rise.
numbers=$(sh "$here/frames.sh" "$dir/capture.raw")
echo "captured frames:" $numbers
[ "$(echo "$numbers" | wc -l)" = 20 ]
last=-1
for n in $numbers; do
    [ "$n" -gt "$last" ] || { echo "frame $n after frame $last"; exit 1; }
    last=$n
done
violations=$($ctl status | sed -n 's/^violations=//p')
$ctl quit
wait "$server" || true
[ "$violations" = 0 ]
echo "video check passed; files in $dir"
