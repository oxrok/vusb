#!/bin/sh
# Reads a raw capture from the test pattern camera (uvc-cam) and prints
# the number each frame shows, one a line. Fails when a frame is not the
# pattern byte for byte: 160x120 YUY2, eight colour bars under eight
# lines that hold the frame number in binary, sixteen bits of ten pixels.
#
#   tools/frames.sh CAPTURE.raw
set -eu
# Ten pixels, five YUYV groups, of the colour $1 $2 $3 (Y U V, in hex).
ten() { printf '%s' "$1$2$1$3" "$1$2$1$3" "$1$2$1$3" "$1$2$1$3" "$1$2$1$3"; }
one=$(ten eb 80 80)
zero=$(ten 10 80 80)
# A line of bars, each twenty pixels: white, yellow, cyan, green,
# magenta, red, blue, black.
bars=
for colour in "b4 80 80" "a2 2c 8e" "83 9c 2c" "70 48 3a" "54 b8 c6" "41 64 d4" "23 d4 72" "10 80 80"; do
    # shellcheck disable=SC2086
    bars=$bars$(ten $colour)$(ten $colour)
done
# Says where line $1 leaves what it should be, $2, and fails.
differs() {
    [ "${#1}" = 640 ] || { echo "$3: the line has $((${#1} / 2)) bytes" >&2; exit 1; }
    at=1
    while [ "$at" -lt 640 ]; do
        got=$(printf '%s' "$1" | cut -c"$at-$((at + 1))")
        want=$(printf '%s' "$2" | cut -c"$at-$((at + 1))")
        [ "$got" = "$want" ] || {
            echo "$3: byte $((at / 2)) is $got, not $want" >&2
            exit 1
        }
        at=$((at + 2))
    done
}
size=$(wc -c <"$1")
[ "$size" -gt 0 ] && [ $((size % 38400)) = 0 ] || {
    echo "$1: $size bytes is not a number of 38400 byte frames" >&2
    exit 1
}
row=0
frame=0
strip=
# The capture in hex, a line of the picture on each line. od's own line
# width is not used: some versions of od break a long line where they
# read on.
{
    od -An -v -tx1 "$1" | tr -d ' \n' | fold -w 640
    echo
} | while read -r line; do
    if [ "$row" = 0 ]; then
        strip=$line
    elif [ "$row" -lt 8 ]; then
        [ "$line" = "$strip" ] || differs "$line" "$strip" "frame $frame, line $row, a copy of line 0"
    else
        [ "$line" = "$bars" ] || differs "$line" "$bars" "frame $frame, line $row, the bars"
    fi
    row=$((row + 1))
    if [ "$row" = 120 ]; then
        bits=$(printf '%s' "$strip" | sed -e "s/$one/1/g" -e "s/$zero/0/g")
        case $bits in
            *[!01]* | "") echo "frame $frame: the strip is not a number" >&2; exit 1 ;;
        esac
        [ "${#bits}" = 16 ] || { echo "frame $frame: the strip has ${#bits} bits" >&2; exit 1; }
        number=0
        for bit in $(printf '%s' "$bits" | sed 's/./& /g'); do
            number=$((number * 2 + bit))
        done
        echo "$number"
        row=0
        frame=$((frame + 1))
    fi
done
