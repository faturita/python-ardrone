#!/usr/bin/env python3
"""
Record H.264 stream from a Raspberry Pi running:
    libcamera-vid -t 0 --inline -o - | nc -l 5555

The stream is raw H.264 served over TCP.  ffmpeg wraps it into a container
(MP4 or MKV) without re-encoding.  Press Ctrl+C to stop — ffmpeg will
finalize the file cleanly.

Usage:
    python record_stream.py [options]

Examples:
    python record_stream.py --ip 192.168.1.3
    python record_stream.py --ip 192.168.1.3 --output flight.mp4
    python record_stream.py --ip 192.168.1.3 --output flight.mkv --fps 30
"""

import argparse
import shutil
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Record Pi camera TCP stream (libcamera-vid | nc) to a file."
    )
    parser.add_argument(
        "--ip", default="192.168.1.3",
        help="Raspberry Pi IP address (default: 192.168.1.3)"
    )
    parser.add_argument(
        "--port", type=int, default=5555,
        help="TCP port that nc is listening on (default: 5555)"
    )
    parser.add_argument(
        "--output", default="recording.mp4",
        help="Output file path; extension picks the container: .mp4 or .mkv (default: recording.mp4)"
    )
    parser.add_argument(
        "--fps", type=float, default=30.0,
        help="Frame rate to stamp on the output (default: 30).  Must match libcamera-vid --framerate."
    )
    parser.add_argument(
        "--timeout", type=int, default=0,
        help="Stop recording after N seconds (0 = unlimited, default: 0)"
    )
    args = parser.parse_args()

    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        print("Error: ffmpeg not found in PATH.  Install it with: brew install ffmpeg")
        sys.exit(1)

    url = f"tcp://{args.ip}:{args.port}"

    cmd = [
        ffmpeg,
        "-fflags",         "nobuffer",
        "-flags",          "low_delay",
        "-probesize",      "32",
        "-analyzeduration","0",
        "-f",              "h264",
        "-framerate",      str(args.fps),   # tell ffmpeg the input frame rate
        "-i",              url,
        "-c:v",            "copy",          # copy bitstream — no re-encode
    ]

    # Container-specific options
    if args.output.lower().endswith(".mp4"):
        # movflags frag_keyframe: write partial MP4 that's recoverable on abort
        cmd += ["-movflags", "frag_keyframe+empty_moov+default_base_moof"]
    # MKV needs no extra flags — it's natively safe for interrupted streams

    if args.timeout > 0:
        cmd += ["-t", str(args.timeout)]

    cmd += ["-y", args.output]   # -y = overwrite without asking

    print(f"Source  : {url}")
    print(f"Output  : {args.output}  (fps={args.fps})")
    if args.timeout:
        print(f"Duration: {args.timeout} seconds")
    else:
        print("Duration: unlimited — press Ctrl+C to stop")
    print()

    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
        rc = proc.returncode
        if rc not in (0, 255):   # 255 = ffmpeg got SIGINT
            print(f"[record] ffmpeg exited with code {rc}")
            sys.exit(rc)
    except KeyboardInterrupt:
        # Forward SIGINT so ffmpeg can write the file trailer cleanly
        proc.send_signal(signal.SIGINT)
        proc.wait()

    print(f"\nSaved → {args.output}")


if __name__ == "__main__":
    main()
