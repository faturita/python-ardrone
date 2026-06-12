# PROJECT.md

Claude project memory for python-ardrone.

## Overview
Python tools to control Parrot AR.Drone 2.0 quadcopters, plus a web control
server. Legacy Python 2 era library code (libardrone.py et al.) coexists with
newer Python 3 Flask servers.

## Web control servers
- drone_server.py : Flask server for AR.Drone 2.0. AT commands over UDP 5556,
  navdata UDP 5554, H.264 video over TCP 5555 (can come from a Raspberry Pi at
  a separate --drone-video-ip). Decodes video via an ffmpeg subprocess pipe to
  BGR24, serves MJPEG at /video_feed.
- tello_drone_server.py : same program adapted for the DJI/Ryze Tello
  (SDK 2.0). Text commands over UDP to <drone-ip>:8889, telemetry received on
  local UDP 8890, video pushed by the drone as raw H.264 to local UDP 11111
  (--drone-ip and --video-port are configurable). Adds Tello-only endpoints:
  /api/flip, /api/streamon, /api/streamoff, /api/status. A 5 s keepalive
  watchdog re-sends "command" because the Tello auto-lands after 15 s of
  command silence. --no-video skips the ffmpeg capture (UDP video port left
  free for an external ffplay viewer; /video_feed shows NO SIGNAL); only one
  process can bind the video port at a time.
- Hardware note (confirmed 2026-06): the user's drone is a standard Tello on
  SDK 1.3/2.0 firmware. setresolution/setbitrate/setfps are SDK 3.0 only
  (Tello TT/EDU) and reply "unknown command"; cmd_init sends them anyway on
  purpose (harmless, and they activate if pointed at an EDU/TT). The stream
  is fixed at 720p; resolution reduction happens only receiver-side. Video
  latency floor (~150-300 ms encode+WiFi) is inherent to the drone.

drone_server.py uses templates/index.html; tello_drone_server.py uses
templates/tello.html, a copy with Tello branding and a 4-direction flip pad
(under Alt/Yaw) that POSTs to /api/flip. Both UIs send /api/move
with lr/fb/vv/va in [-1..1] using the AR.Drone sign convention (fb negative =
forward); the Tello server negates fb when mapping to "rc a b c d" (-100..100).

## Conventions
- ASCII hyphen-minus only; no Unicode dashes in code or comments.
- New code mirrors drone_server.py structure (module-level command engine,
  VideoStream class, Flask routes, argparse entry point).
