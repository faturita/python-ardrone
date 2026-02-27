"""
Flask-based AR.Drone 2.0 Control Server
Sends AT commands directly over UDP — no libardrone dependency.

Usage:
    python drone_server.py [--drone-ip 192.168.x.x] [--port 5000]
"""

import socket
import struct
import time
import threading
import argparse
import subprocess
import shutil
import cv2
import numpy as np
from flask import Flask, Response, render_template, jsonify, request

# ─── Configuration ────────────────────────────────────────────────────────────

DEFAULT_DRONE_IP   = "192.168.1.1"
DRONE_CMD_PORT     = 5556          # AT commands (UDP)
DRONE_NAVDATA_PORT = 5554          # Navdata (UDP)
DRONE_VIDEO_PORT   = 5555          # Video UDP trigger / TCP stream
MOVE_SPEED         = 0.3           # default movement speed  [0..1]
WATCHDOG_INTERVAL  = 0.5           # seconds between keepalives

# ─── Flask app ────────────────────────────────────────────────────────────────

app = Flask(__name__)

# ─── AT Command Engine ────────────────────────────────────────────────────────

_seq_nr    = 1
_seq_lock  = threading.Lock()
_wdg_timer = None
_cmd_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_drone_ip  = DEFAULT_DRONE_IP


def f2i(f: float) -> int:
    """Reinterpret a float's IEEE-754 bytes as a signed 32-bit integer."""
    return struct.unpack('i', struct.pack('f', f))[0]


def _build_at(command: str, seq: int, params: list) -> bytes:
    """Encode a single AT* command string."""
    parts = []
    for p in params:
        if isinstance(p, bool):           # bool before int — Python bool is int subclass
            parts.append(",%d" % int(p))
        elif isinstance(p, int):
            parts.append(",%d" % p)
        elif isinstance(p, float):
            parts.append(",%d" % f2i(p))
        elif isinstance(p, str):
            parts.append(',"' + p + '"')
    return ("AT*%s=%d%s\r" % (command, seq, "".join(parts))).encode("utf-8")


def send_at(command: str, params: list):
    """Thread-safe AT command sender with auto sequence-numbering + watchdog reset."""
    global _seq_nr, _wdg_timer
    with _seq_lock:
        msg = _build_at(command, _seq_nr, params)
        _cmd_sock.sendto(msg, (_drone_ip, DRONE_CMD_PORT))
        _seq_nr += 1
        # Reset communication watchdog
        if _wdg_timer:
            _wdg_timer.cancel()
        _wdg_timer = threading.Timer(WATCHDOG_INTERVAL, _comwdg_fire)
        _wdg_timer.daemon = True
        _wdg_timer.start()


def _comwdg_fire():
    """Send watchdog keepalive (runs in a timer thread)."""
    send_at("COMWDG", [])


# ─── High-level drone commands ────────────────────────────────────────────────

def _ref(*, takeoff: bool = False, emergency: bool = False) -> int:
    """Compute the AT*REF integer value from the AR.Drone SDK spec."""
    base = 0b10001010101000000000000000000   # = 290 717 696
    if takeoff:
        base |= (1 << 9)
    if emergency:
        base |= (1 << 8)
    return base


def cmd_init():
    """
    Bootstrap sequence (mirrors what arnetwork.py and the FreeFlight app do):

    1. Send UDP trigger \x01\x00\x00\x00 to navdata port 5554 — wakes the
       drone's navdata subsystem out of bootstrap mode.
    2. Send UDP trigger \x01\x00\x00\x00 to video port 5555 — tells the drone
       to start the H.264 TCP stream on that port.
    3. Configure navdata demo mode via AT*CONFIG.
    4. Configure H.264 360p video codec (value 129 per AR.Drone SDK).
    5. Flat-trim.
    """
    trigger = b"\x01\x00\x00\x00"

    nav_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    nav_sock.sendto(trigger, (_drone_ip, DRONE_NAVDATA_PORT))
    nav_sock.close()

    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vid_sock.sendto(trigger, (_drone_ip, DRONE_VIDEO_PORT))
    vid_sock.close()

    time.sleep(0.1)

    send_at("CONFIG", ["general:navdata_demo", "TRUE"])
    send_at("CONFIG", ["general:video_enable", "TRUE"])   # enable video stream
    send_at("CONFIG", ["general:vision_enable", "TRUE"])  # enable vision/video subsystem
    send_at("CONFIG", ["video:video_codec", "129"])        # H264_360P
    send_at("CONFIG", ["video:video_channel", "1"])        # 0=front camera  1=belly camera
    send_at("FTRIM", [])


def cmd_takeoff():
    send_at("FTRIM", [])
    send_at("CONFIG", ["control:altitude_max", "20000"])
    send_at("REF", [_ref(takeoff=True)])


def cmd_land():
    send_at("REF", [_ref(takeoff=False)])


def cmd_emergency():
    """Cut motors immediately by toggling the emergency bit."""
    send_at("REF", [_ref(emergency=True)])


def cmd_reset():
    """Clear emergency state so the drone can fly again."""
    send_at("REF", [_ref(emergency=True)])
    time.sleep(0.05)
    send_at("REF", [_ref(emergency=False)])


def cmd_hover():
    """Stop all movement — send progressive=0 (hover mode)."""
    send_at("PCMD", [0, float(0), float(0), float(0), float(0)])


def cmd_move(lr: float = 0.0, fb: float = 0.0,
             vv: float = 0.0, va: float = 0.0):
    """
    Progressive movement command.

    lr: left/right tilt  [-1..1]  negative=left,     positive=right
    fb: front/back tilt  [-1..1]  negative=forward,  positive=backward
    vv: vertical speed   [-1..1]  negative=down,     positive=up
    va: angular speed    [-1..1]  negative=spin-left, positive=spin-right
    """
    send_at("PCMD", [1, float(lr), float(fb), float(vv), float(va)])


# ─── Video Streaming ──────────────────────────────────────────────────────────

# AR.Drone 2.0 default stream resolution
_VID_W, _VID_H = 640, 360


class VideoStream:
    """
    Captures H.264 frames from the AR.Drone via an ffmpeg subprocess pipe.

    The drone serves raw H.264 over HTTP on port 5555 (no container).
    OpenCV's VideoCapture cannot auto-detect this, so we replicate the
    flags used in seevideo.sh:
        ffmpeg -framedrop -infbuf -f h264 -i http://<ip>:5555 ...
    and pipe decoded BGR frames into numpy arrays.
    """

    _NO_SIGNAL: bytes | None = None

    def __init__(self, drone_ip: str):
        self.drone_ip = drone_ip
        self._frame   = None
        self._lock    = threading.Lock()
        self._thread  = threading.Thread(target=self._capture, daemon=True)

    def start(self):
        self._thread.start()

    def _capture(self):
        url         = f"http://{self.drone_ip}:5555"
        frame_bytes = _VID_W * _VID_H * 3   # BGR24

        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            print("[video] ffmpeg not found — falling back to OpenCV VideoCapture")
            self._capture_opencv(url)
            return

        cmd = [
            ffmpeg,
            "-loglevel", "warning",
            "-fflags", "nobuffer",      # disable input buffering
            "-flags", "low_delay",      # low-latency decode
            "-probesize", "32",         # minimal format probing
            "-analyzeduration", "0",    # skip analysis delay
            "-f", "h264",               # force raw H.264 input format
            "-i", url,
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-vf", f"scale={_VID_W}:{_VID_H}",
            "pipe:1",
        ]

        while True:
            print(f"[video] connecting to {url} …")
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=None,        # inherit stderr so ffmpeg errors show in console
                    bufsize=frame_bytes * 4,
                )
                buf = b""
                while True:
                    chunk = proc.stdout.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    # Consume every complete frame in the buffer
                    while len(buf) >= frame_bytes:
                        arr = np.frombuffer(buf[:frame_bytes], dtype=np.uint8)
                        with self._lock:
                            self._frame = arr.reshape(_VID_H, _VID_W, 3).copy()
                        buf = buf[frame_bytes:]
                proc.stdout.close()
                proc.wait()
            except Exception as exc:
                print(f"[video] error: {exc}")
            print("[video] stream ended — retrying in 2 s")
            time.sleep(2)

    def _capture_opencv(self, url: str):
        """Fallback path when ffmpeg binary is unavailable."""
        while True:
            try:
                cap = cv2.VideoCapture(url)
                while cap.isOpened():
                    ok, frame = cap.read()
                    if ok:
                        with self._lock:
                            self._frame = frame
                cap.release()
            except Exception:
                pass
            time.sleep(2)

    def get_jpeg(self) -> bytes:
        with self._lock:
            frame = self._frame
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                return buf.tobytes()
        return self._no_signal()

    def _no_signal(self) -> bytes:
        if VideoStream._NO_SIGNAL is None:
            img = np.zeros((_VID_H, _VID_W, 3), dtype=np.uint8)
            img[:] = (20, 20, 30)
            cv2.putText(img, "NO SIGNAL", (165, 195),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.2, (60, 80, 60), 3,
                        cv2.LINE_AA)
            cv2.putText(img, "drone not connected", (195, 245),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 70, 50), 1,
                        cv2.LINE_AA)
            _, buf = cv2.imencode(".jpg", img)
            VideoStream._NO_SIGNAL = buf.tobytes()
        return VideoStream._NO_SIGNAL


_video: VideoStream | None = None


def _gen_mjpeg():
    """MJPEG frame generator for Flask streaming response."""
    while True:
        jpeg = _video.get_jpeg()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n"
               + jpeg + b"\r\n")
        time.sleep(1 / 25)   # ~25 fps


# ─── Flask Routes ─────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html", drone_ip=_drone_ip)


@app.route("/video_feed")
def video_feed():
    return Response(_gen_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/takeoff", methods=["POST"])
def api_takeoff():
    cmd_takeoff()
    return jsonify(ok=True, cmd="takeoff")


@app.route("/api/land", methods=["POST"])
def api_land():
    cmd_land()
    return jsonify(ok=True, cmd="land")


@app.route("/api/emergency", methods=["POST"])
def api_emergency():
    cmd_emergency()
    return jsonify(ok=True, cmd="emergency")


@app.route("/api/reset", methods=["POST"])
def api_reset():
    cmd_reset()
    return jsonify(ok=True, cmd="reset")


@app.route("/api/hover", methods=["POST"])
def api_hover():
    cmd_hover()
    return jsonify(ok=True, cmd="hover")


@app.route("/api/move", methods=["POST"])
def api_move():
    data = request.get_json(force=True) or {}
    lr = float(data.get("lr", 0))
    fb = float(data.get("fb", 0))
    vv = float(data.get("vv", 0))
    va = float(data.get("va", 0))
    cmd_move(lr=lr, fb=fb, vv=vv, va=va)
    return jsonify(ok=True, cmd="move", lr=lr, fb=fb, vv=vv, va=va)


# ─── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AR.Drone 2.0 Flask Control Server")
    parser.add_argument("--drone-ip", default=DEFAULT_DRONE_IP,
                        help="Drone IP address (default: 192.168.1.1)")
    parser.add_argument("--port", type=int, default=8000,
                        help="HTTP server port (default: 8000)")
    args = parser.parse_args()

    _drone_ip = args.drone_ip
    video_url = f"http://{_drone_ip}:5555"

    print(f"  Drone IP : {_drone_ip}")
    print(f"  Video    : {video_url}  (raw H.264 via ffmpeg)")
    print(f"  Server   : http://0.0.0.0:{args.port}")

    # Initialize drone FIRST (sends UDP triggers + AT config)
    # so the drone starts the H.264 stream before ffmpeg connects.
    cmd_init()
    time.sleep(0.5)

    _video = VideoStream(_drone_ip)
    _video.start()

    app.run(host="0.0.0.0", port=args.port, debug=False, threaded=True)
