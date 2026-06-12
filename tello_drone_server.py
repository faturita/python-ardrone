"""
Flask-based DJI/Ryze Tello Control Server
Sends Tello SDK text commands directly over UDP - no external tello library.

Protocol (Tello SDK 2.0):
  - Commands : UDP text to <drone-ip>:8889  ("command", "takeoff", "rc a b c d", ...)
  - State    : drone pushes telemetry to local UDP port 8890
  - Video    : after "streamon" the drone pushes a raw H.264 stream to the
               client on UDP port 11111 (we listen on udp://0.0.0.0:<video-port>)

Usage:
    python tello_drone_server.py [--drone-ip 192.168.10.1] [--video-port 11111] [--port 8000]
"""

import socket
import time
import threading
import argparse
import subprocess
import shutil
import cv2
import numpy as np
from flask import Flask, Response, render_template, jsonify, request

# --- Configuration ------------------------------------------------------------

DEFAULT_DRONE_IP   = "192.168.10.1"
DEFAULT_VIDEO_PORT = 11111         # drone pushes H.264 here (UDP, local listen)
DRONE_CMD_PORT     = 8889          # Tello SDK commands (UDP)
DRONE_STATE_PORT   = 8890          # Telemetry pushed by the drone (UDP, local listen)
MOVE_SPEED         = 0.3           # default movement speed  [0..1]
KEEPALIVE_INTERVAL = 5.0           # Tello auto-lands after 15 s without a command

# Video display resolution - overridden by --video-width / --video-height
# Kept low by default: smaller raw frames keep the decode pipe from backlogging
# (Tello camera native is 960x720)
_VID_W, _VID_H = 480, 360

# --- Flask app ------------------------------------------------------------------

app = Flask(__name__)

# --- Tello Command Engine -------------------------------------------------------

_cmd_lock   = threading.Lock()
_wdg_timer  = None
_cmd_sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_cmd_sock.bind(("", 9000))         # local port for command responses
_drone_ip   = DEFAULT_DRONE_IP
_video_port = DEFAULT_VIDEO_PORT
_last_response = None


def send_cmd(command: str):
    """Thread-safe Tello command sender with keepalive watchdog reset."""
    global _wdg_timer
    with _cmd_lock:
        _cmd_sock.sendto(command.encode("utf-8"), (_drone_ip, DRONE_CMD_PORT))
        # Reset keepalive watchdog (Tello lands after 15 s of silence)
        if _wdg_timer:
            _wdg_timer.cancel()
        _wdg_timer = threading.Timer(KEEPALIVE_INTERVAL, _keepalive_fire)
        _wdg_timer.daemon = True
        _wdg_timer.start()


def _keepalive_fire():
    """Send keepalive so the Tello does not auto-land (runs in a timer thread)."""
    send_cmd("command")


def _response_listener():
    """Read command responses ("ok" / "error") from the command socket."""
    global _last_response
    while True:
        try:
            data, _ = _cmd_sock.recvfrom(1024)
            _last_response = data.decode("utf-8", errors="replace").strip()
            print(f"[tello] response: {_last_response}")
        except Exception as exc:
            print(f"[tello] response listener error: {exc}")
            time.sleep(1)


# --- Telemetry (state port 8890) -------------------------------------------------

_state      = {}
_state_lock = threading.Lock()


def _state_listener():
    """
    Parse telemetry pushed by the drone to UDP 8890, e.g.:
        pitch:0;roll:0;yaw:0;vgx:0;...;bat:87;baro:163.74;...;agx:0.00;...
    """
    global _state
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", DRONE_STATE_PORT))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            fields = {}
            for pair in data.decode("utf-8", errors="replace").strip().split(";"):
                if ":" in pair:
                    key, value = pair.split(":", 1)
                    fields[key] = value
            with _state_lock:
                _state = fields
        except Exception as exc:
            print(f"[tello] state listener error: {exc}")
            time.sleep(1)


# --- High-level drone commands ----------------------------------------------------

def cmd_init():
    """
    Bootstrap sequence (per the Tello SDK):

    1. "command"  - enter SDK mode (required before any other command).
    2. video tuning for low latency (SDK 3.0 commands; older firmware just
       replies "unknown command", which is harmless).
    3. "streamon" - start the H.264 video stream on UDP 11111.
    """
    send_cmd("command")
    time.sleep(0.5)
    send_cmd("setresolution low")   # 480p stream instead of 720p
    time.sleep(0.1)
    send_cmd("setbitrate 2")        # cap at 2 Mbps
    time.sleep(0.1)
    send_cmd("streamon")


def cmd_takeoff():
    send_cmd("takeoff")


def cmd_land():
    send_cmd("land")


def cmd_emergency():
    """Cut motors immediately."""
    send_cmd("emergency")


def cmd_reset():
    """Re-enter SDK mode after an emergency so the drone can fly again."""
    send_cmd("command")


def cmd_hover():
    """Stop all movement - zero out the rc channels."""
    send_cmd("rc 0 0 0 0")


def cmd_move(lr: float = 0.0, fb: float = 0.0,
             vv: float = 0.0, va: float = 0.0):
    """
    Continuous movement via "rc a b c d" (each channel -100..100).

    Inputs keep the AR.Drone web-UI convention used by index.html:
    lr: left/right        [-1..1]  negative=left,      positive=right
    fb: front/back        [-1..1]  negative=forward,   positive=backward
    vv: vertical speed    [-1..1]  negative=down,      positive=up
    va: angular speed     [-1..1]  negative=spin-left, positive=spin-right

    Tello rc: a=roll(right+), b=pitch(forward+), c=throttle(up+), d=yaw(cw+),
    so fb is negated.
    """
    a = int(max(-1.0, min(1.0, lr)) * 100)
    b = int(max(-1.0, min(1.0, -fb)) * 100)
    c = int(max(-1.0, min(1.0, vv)) * 100)
    d = int(max(-1.0, min(1.0, va)) * 100)
    send_cmd(f"rc {a} {b} {c} {d}")


def cmd_flip(direction: str):
    """Flip in a direction: l=left  r=right  f=forward  b=back."""
    send_cmd(f"flip {direction}")


# --- Video Streaming -----------------------------------------------------------

class VideoStream:
    """
    Captures H.264 frames from the Tello UDP stream via an ffmpeg subprocess pipe.

    After "streamon" the drone pushes raw H.264 over UDP to port 11111 on the
    client, so ffmpeg listens on udp://0.0.0.0:<video-port>, decodes frames,
    and pipes BGR24 raw video into numpy arrays for MJPEG serving.
    """

    _NO_SIGNAL: bytes | None = None

    def __init__(self, video_port: int):
        self.video_port = video_port
        self._frame   = None
        self._lock    = threading.Lock()
        self._thread  = threading.Thread(target=self._capture, daemon=True)

    def start(self):
        self._thread.start()

    def _capture(self):
        # The Tello pushes the stream to us, so listen on all interfaces.
        # overrun_nonfatal: drop packets instead of dying when we fall behind;
        # fifo_size is in 188-byte units (~1 MB) - small to limit input buffering.
        url         = (f"udp://0.0.0.0:{self.video_port}"
                       f"?overrun_nonfatal=1&fifo_size=5000")
        frame_bytes = _VID_W * _VID_H * 3   # BGR24

        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            print("[video] ffmpeg not found - falling back to OpenCV VideoCapture")
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
            print(f"[video] listening on {url} ...")
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=None,        # inherit stderr so ffmpeg errors show in console
                    bufsize=frame_bytes * 4,
                )
                buf = b""
                while True:
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    # Keep only the NEWEST complete frame; older ones are
                    # stale and replaying them just adds latency.
                    n = len(buf) // frame_bytes
                    if n:
                        start = (n - 1) * frame_bytes
                        arr = np.frombuffer(buf[start:start + frame_bytes],
                                            dtype=np.uint8)
                        with self._lock:
                            self._frame = arr.reshape(_VID_H, _VID_W, 3).copy()
                        buf = buf[n * frame_bytes:]
                proc.stdout.close()
                proc.wait()
            except Exception as exc:
                print(f"[video] error: {exc}")
            print("[video] stream ended - retrying in 2 s")
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


# --- Flask Routes ----------------------------------------------------------------

@app.route("/")
def index():
    return render_template("tello.html", drone_ip=_drone_ip)


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


# --- Tello-specific extras ---------------------------------------------------------

@app.route("/api/flip", methods=["POST"])
def api_flip():
    """Flip: pass {"direction": "l"|"r"|"f"|"b"}."""
    data = request.get_json(force=True) or {}
    direction = data.get("direction", "f")
    if direction not in ("l", "r", "f", "b"):
        return jsonify(ok=False, error="direction must be l, r, f or b"), 400
    cmd_flip(direction)
    return jsonify(ok=True, cmd="flip", direction=direction)


@app.route("/api/streamon", methods=["POST"])
def api_streamon():
    send_cmd("streamon")
    return jsonify(ok=True, cmd="streamon")


@app.route("/api/streamoff", methods=["POST"])
def api_streamoff():
    send_cmd("streamoff")
    return jsonify(ok=True, cmd="streamoff")


@app.route("/api/status")
def api_status():
    """Latest telemetry pushed by the drone (battery, height, attitude, ...)."""
    with _state_lock:
        state = dict(_state)
    return jsonify(ok=True, state=state, last_response=_last_response)


# --- Entry point ----------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tello Flask Control Server")
    parser.add_argument("--drone-ip", default=DEFAULT_DRONE_IP,
                        help="Drone IP address (default: 192.168.10.1)")
    parser.add_argument("--video-port", type=int, default=DEFAULT_VIDEO_PORT,
                        help="UDP port the drone streams video to (default: 11111)")
    parser.add_argument("--video-width", type=int, default=_VID_W,
                        help=f"Video display width in pixels (default: {_VID_W})")
    parser.add_argument("--video-height", type=int, default=_VID_H,
                        help=f"Video display height in pixels (default: {_VID_H})")
    parser.add_argument("--no-video", action="store_true",
                        help="Do not capture video; leaves the UDP video port "
                             "free so an external player (ffplay) can use it")
    parser.add_argument("--port", type=int, default=8000,
                        help="HTTP server port (default: 8000)")
    args = parser.parse_args()

    _drone_ip = args.drone_ip
    _video_port = args.video_port
    _VID_W = args.video_width
    _VID_H = args.video_height
    video_url = f"udp://0.0.0.0:{_video_port}"

    print(f"  Drone IP : {_drone_ip}")
    if args.no_video:
        print(f"  Video    : disabled (--no-video). Watch externally with:")
        print(f"             ffplay -fflags nobuffer -flags low_delay "
              f"-framedrop -sync ext -probesize 32 -analyzeduration 0 "
              f"-vf setpts=0 \"udp://0.0.0.0:{_video_port}\"")
    else:
        print(f"  Video    : {video_url}  ({_VID_W}x{_VID_H}, raw H.264 via ffmpeg)")
    print(f"  Server   : http://0.0.0.0:{args.port}")

    # Listener threads for command responses and telemetry
    threading.Thread(target=_response_listener, daemon=True).start()
    threading.Thread(target=_state_listener, daemon=True).start()

    # Initialize drone FIRST ("command" + "streamon")
    # so the drone starts pushing the H.264 stream before ffmpeg listens.
    cmd_init()
    time.sleep(0.5)

    # With --no-video the VideoStream is created but never started: the UDP
    # video port stays free for ffplay and /video_feed serves NO SIGNAL.
    _video = VideoStream(_video_port)
    if not args.no_video:
        _video.start()

    app.run(host="0.0.0.0", port=args.port, debug=False, threaded=True)
