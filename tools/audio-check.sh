#!/bin/sh
# Attaches the tone microphone through vhci-hcd and records from it with
# the kernel's snd-usb-audio: an isochronous stream, checked sample by
# sample, first as fast as the host asks and then in real time. Needs
# root, vhci-hcd, snd-usb-audio and arecord.
set -eu
dir=$(mktemp -d)
socket=$dir/c.sock
ctl="target/debug/vusb-ctl -s $socket"
target/debug/vusb run uac-mic --attach --listen 127.0.0.1:0 \
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
tries=0
until card=$(sed -n 's/^ *\([0-9]*\) .*Tone microphone.*/\1/p' /proc/asound/cards | head -1) &&
    [ -n "$card" ]; do
    tries=$((tries + 1))
    [ "$tries" -gt 100 ] && { echo "no sound card for the microphone"; cat /proc/asound/cards; exit 1; }
    sleep 0.1
done
# Records two seconds of sound and checks every sample; how long the
# recording took, in milliseconds, is left in $took.
record() {
    began=$(date +%s%N)
    arecord -q -D "hw:$card,0" -f S16_LE -r 48000 -c 1 -d 2 -t raw "$dir/rec.raw"
    took=$((($(date +%s%N) - began) / 1000000))
    size=$(wc -c <"$dir/rec.raw")
    start=$(od -An -tx1 -N 16 "$dir/rec.raw" | tr -d ' \n')
    echo "recorded $size bytes in $took ms, starting $start"
    # Two seconds at 48 kHz, 16 bit; every 1 ms packet is one period of
    # the 1 kHz tone, so the recording starts as the tone does.
    [ "$size" = 192000 ]
    [ "$start" = 00005b0890107e180020f626412dc632 ]
    # Every period is the same 96 bytes: the whole recording is the tone.
    periods=$(od -An -v -tx1 "$dir/rec.raw" | tr -d ' \n' | fold -w 192 | sort -u | wc -l)
    echo "distinct periods: $periods"
    [ "$periods" = 1 ]
}
# As fast as the host asks: the two seconds of sound take much less.
record
# In real time, a fresh model with the option: the same sound takes its
# two seconds, and not much more on a machine that is busy.
$ctl options realtime=1
$ctl reset
record
[ "$took" -ge 1900 ] || { echo "two seconds of sound in $took ms: not in real time"; exit 1; }
[ "$took" -lt 6000 ] || { echo "two seconds of sound took $took ms"; exit 1; }
violations=$($ctl status | sed -n 's/^violations=//p')
$ctl quit
wait "$server" || true
[ "$violations" = 0 ]
echo "audio check passed; files in $dir"
