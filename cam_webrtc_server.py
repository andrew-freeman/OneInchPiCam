#!/usr/bin/env python3
import asyncio
import json
import threading
import time
from typing import Optional, Dict, Any

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")
from gi.repository import Gst, GLib, GstWebRTC, GstSdp  # type: ignore

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Body
from fastapi.responses import HTMLResponse, JSONResponse

Gst.init(None)

# ----------------------------
# GStreamer / Camera Controller
# ----------------------------

class CameraApp:
    """
    Owns a single GStreamer pipeline:
      libcamerasrc -> NV12 -> openh264enc -> rtph264pay -> webrtcbin

    Provides thread-safe control (AE/exposure/gain) via GLib idle callbacks,
    live stats via identity handoff, and WebRTC signalling hooks.
    """

    def __init__(self):
        self.mainloop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self._run_glib, daemon=True)

        self.pipeline: Optional[Gst.Pipeline] = None
        self.cam: Optional[Gst.Element] = None
        self.stats_id: Optional[Gst.Element] = None
        self.webrtc: Optional[Gst.Element] = None

        self._lock = threading.RLock()
        self._asyncio_loop = None

        # Stats
        self._frames = 0
        self._t0 = time.monotonic()
        self._fps = 0.0
        self._last_stats = {
            "fps": 0.0,
            "ae_enable": None,
            "exposure_time_us": None,
            "analogue_gain": None,
            "ts": time.time(),
        }

        # WebRTC signalling
        self._ws_signalling: Optional[WebSocket] = None
        self._negotiation_started = False

        # Configurable defaults
        self.width = 1280
        self.height = 720
        self.fps = 30
        self.bitrate_bps = 4_000_000  # openh264enc expects bps-ish scale in some builds; we’ll keep it consistent
        self.complexity = 0           # lowest CPU, good for realtime

    def start(self):
        self.loop_thread.start()
        # Build pipeline inside GLib thread so GStreamer is fully happy.
        self._glib_call(self._build_and_start_pipeline)

    def stop(self):
        self._glib_call(self._stop_pipeline)
        if self.mainloop.is_running():
            self.mainloop.quit()

    # ---- GLib thread helpers ----

    def _run_glib(self):
        self.mainloop.run()

    def _glib_call(self, fn, *args, **kwargs):
        # Run a function in GLib main context (thread-safe interaction with GStreamer)
        done = threading.Event()
        result = {"exc": None}

        def _wrap():
            try:
                fn(*args, **kwargs)
            except Exception as e:
                result["exc"] = e
            finally:
                done.set()
            return False

        GLib.idle_add(_wrap)
        done.wait()
        if result["exc"]:
            raise result["exc"]

    # ---- Pipeline lifecycle ----

    def _pipeline_str(self) -> str:
        return (
            f"libcamerasrc name=cam ! "
            f"video/x-raw,format=NV12,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            f"identity name=stats signal-handoffs=true ! "
            f"queue ! videoconvert ! "
            f"openh264enc bitrate={self.bitrate_bps} complexity={self.complexity} ! "
            f"h264parse config-interval=1 ! "
            f"rtph264pay pt=96 config-interval=1 ! "
            f"webrtcbin name=webrtc bundle-policy=max-bundle "
        )

    def _build_and_start_pipeline(self):
        with self._lock:
            self._stop_pipeline()

            desc = self._pipeline_str()
            self.pipeline = Gst.parse_launch(desc)
            assert isinstance(self.pipeline, Gst.Pipeline)

            self.cam = self.pipeline.get_by_name("cam")
            self.stats_id = self.pipeline.get_by_name("stats")
            self.webrtc = self.pipeline.get_by_name("webrtc")

            if not self.cam or not self.stats_id or not self.webrtc:
                raise RuntimeError("Failed to locate required elements (cam/stats/webrtc).")

            # Identity handoff for FPS (GI signature varies across builds → normalize)
            self.stats_id.connect("handoff", self._on_handoff_any)

            # WebRTC signals
            self.webrtc.connect("on-negotiation-needed", self._on_negotiation_needed)
            self.webrtc.connect("on-ice-candidate", self._on_ice_candidate)

            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_bus_message)

            self._frames = 0
            self._t0 = time.monotonic()
            self._fps = 0.0
            self._negotiation_started = False

            self.pipeline.set_state(Gst.State.PLAYING)


    def _on_handoff_any(self, identity, buffer, *args):
        """
        Normalizes GStreamer identity::handoff callback across GI builds.

        Some builds call: (identity, buffer, pad)
        Others call:      (identity, buffer)
        """
        pad = args[0] if args else None
        self._on_handoff(identity, buffer, pad)



    def _stop_pipeline(self):
        with self._lock:
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
                self.cam = None
                self.stats_id = None
                self.webrtc = None
            self._negotiation_started = False

    def restart_with_fps(self, fps: int):
        with self._lock:
            self.fps = int(max(1, min(120, fps)))
        self._build_and_start_pipeline()

    # ---- Stats ----
    def _on_handoff(self, element, buffer, pad=None):
        self._frames += 1
        now = time.monotonic()
        dt = now - self._t0

        if dt >= 0.5:
            fps = self._frames / dt
            self._frames = 0
            self._t0 = now

            ae = self._safe_get_property(self.cam, "ae-enable")
            exp = self._safe_get_property(self.cam, "exposure-time")
            gain = self._safe_get_property(self.cam, "analogue-gain")

            self._last_stats = {
                "fps": round(fps, 2),
                "ae_enable": ae,
                "exposure_time_us": exp,
                "analogue_gain": gain,
                "ts": time.time(),
            }




    def _safe_get_property(self, elem: Optional[Gst.Element], prop: str):
        if not elem:
            return None
        try:
            return elem.get_property(prop)
        except Exception:
            return None

    def get_stats(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._last_stats)

    # ---- Camera controls (thread-safe wrappers) ----

    def set_ae(self, enabled: bool):
        def _set():
            if self.cam:
                self.cam.set_property("ae-enable", bool(enabled))
        self._glib_call(_set)

    def set_exposure_us(self, us: int):
        # Exposure only takes effect if exposure-time-mode/manual is used on some stacks.
        # On many Pi pipelines, setting exposure-time while AE is enabled might be ignored.
        # We’ll set AE off for deterministic behavior, but you can change that later.
        us = int(max(1, us))

        def _set():
            if self.cam:
                # If your build supports exposure-time-mode, you can also set it:
                # self.cam.set_property("exposure-time-mode", "manual")
                try:
                    self.cam.set_property("ae-enable", False)
                except Exception:
                    pass
                self.cam.set_property("exposure-time", us)
        self._glib_call(_set)

    def set_gain(self, gain: float):
        gain = float(max(1.0, gain))

        def _set():
            if self.cam:
                try:
                    self.cam.set_property("ae-enable", False)
                except Exception:
                    pass
                self.cam.set_property("analogue-gain", gain)
        self._glib_call(_set)

    # ---- WebRTC signalling ----

    def attach_signalling_ws(self, ws: WebSocket):
        with self._lock:
            self._ws_signalling = ws
            self._negotiation_started = False

        def _kick():
            if self.webrtc:
                self.webrtc.emit("on-negotiation-needed")

        self._glib_call(_kick)


    def detach_signalling_ws(self, ws: WebSocket):
        with self._lock:
            if self._ws_signalling is ws:
                self._ws_signalling = None
            self._negotiation_started = False

    def _send_ws_json_threadsafe(self, payload: Dict[str, Any]):
        ws = self._ws_signalling
        loop = self._asyncio_loop

        if not ws or not loop:
            return

        asyncio.run_coroutine_threadsafe(
            ws.send_text(json.dumps(payload)),
            loop
        )

    def _on_negotiation_needed(self, webrtcbin):
        with self._lock:
            if self._negotiation_started:
                return
            self._negotiation_started = True

        def on_offer_created(promise: Gst.Promise, _, __):
            reply = promise.get_reply()
            offer = reply.get_value("offer")

            webrtcbin.emit(
                "set-local-description",
                offer,
                Gst.Promise.new()
            )

            self._send_ws_json_threadsafe({
                "type": "offer",
                "sdp": offer.sdp.as_text()
            })

        promise = Gst.Promise.new_with_change_func(on_offer_created, None, None)
        webrtcbin.emit("create-offer", None, promise)


    def _on_ice_candidate(self, webrtcbin, mlineindex, candidate):
        self._send_ws_json_threadsafe({
            "type": "ice",
            "sdpMLineIndex": int(mlineindex),
            "candidate": str(candidate),
        })

    def apply_answer_sdp(self, sdp_text: str):
        def _apply():
            if not self.webrtc:
                return
            res, sdpmsg = GstSdp.sdp_message_new()
            if res != GstSdp.SDPResult.OK:
                raise RuntimeError("Failed to allocate SDP message")

            res = GstSdp.sdp_message_parse_buffer(bytes(sdp_text.encode("utf-8")), sdpmsg)
            if res != GstSdp.SDPResult.OK:
                raise RuntimeError("Failed to parse SDP answer")

            answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
            self.webrtc.emit("set-remote-description", answer, Gst.Promise.new())
        self._glib_call(_apply)

    def add_ice_candidate(self, mlineindex: int, candidate: str):
        def _add():
            if self.webrtc:
                self.webrtc.emit("add-ice-candidate", int(mlineindex), str(candidate))
        self._glib_call(_add)

    # ---- Bus messages ----

    def _on_bus_message(self, bus, message):
        t = message.type

        if t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"[GstError] {err}\n{dbg}")

        elif t == Gst.MessageType.WARNING:
            err, dbg = message.parse_warning()
            print(f"[GstWarn] {err}\n{dbg}")

        elif t == Gst.MessageType.EOS:
            print("[Gst] End of stream")

        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old, new, pending = message.parse_state_changed()
                print(f"[Pipeline] {old.value_nick} → {new.value_nick}")



camera = CameraApp()
camera.start()

# ----------------------------
# FastAPI Web App
# ----------------------------

app = FastAPI()

HTML_PAGE = r"""
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>IMX283 WebRTC Camera</title>
  <style>
    body { font-family: sans-serif; margin: 12px; }
    .row { display:flex; gap: 12px; flex-wrap: wrap; }
    .card { border: 1px solid #ccc; border-radius: 10px; padding: 12px; }
    video { width: 100%; max-width: 960px; background: #000; border-radius: 10px; }
    label { display:block; margin-top: 8px; }
    input[type=range] { width: 320px; }
    code { background:#f5f5f5; padding: 2px 6px; border-radius: 6px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
  </style>
</head>
<body>
  <h2>IMX283 WebRTC Camera</h2>

  <div class="card">
    <video id="v" autoplay playsinline muted></video>
    <div id="status" class="mono" style="margin-top:8px;">status: idle</div>
  </div>

  <div class="row">
    <div class="card">
      <h3>Controls</h3>

      <button onclick="postJSON('/api/ae', {enabled:true})">AE ON</button>
      <button onclick="postJSON('/api/ae', {enabled:false})">AE OFF</button>

      <label>Exposure (µs): <span id="expv">20000</span></label>
      <input id="exp" type="range" min="100" max="50000" step="100" value="20000"/>

      <label>Gain: <span id="gainv">4.0</span></label>
      <input id="gain" type="range" min="1.0" max="16.0" step="0.1" value="4.0"/>

      <label>FPS (restart): <span id="fpsv">30</span></label>
      <input id="fps" type="range" min="5" max="60" step="1" value="30"/>
      <button onclick="applyFps()">Apply FPS (restart pipeline)</button>

      <div style="margin-top:10px;">
        <button onclick="connect()">Connect WebRTC</button>
        <button onclick="disconnect()">Disconnect</button>
      </div>

      <p style="margin-top:12px;">
        If video doesn’t appear, check <code>gst-inspect-1.0 webrtcbin</code> and Pi firewall.
      </p>
    </div>

    <div class="card">
      <h3>Live Stats</h3>
      <div class="mono" id="stats">waiting...</div>
    </div>
  </div>

<script>
let pc = null;
let ws = null;
let wsStats = null;

async function postJSON(url, obj) {
  await fetch(url, {
    method: "POST",
    headers: {"Content-Type":"application/json"},
    body: JSON.stringify(obj)
  });
}

function setStatus(s){ document.getElementById("status").textContent = "status: " + s; }

async function connect() {
  if (pc) return;

  setStatus("connecting...");

  // Ensure stats start even if user only wants stats
  connectStats();

  pc = new RTCPeerConnection({
    iceServers: [{urls: "stun:stun.l.google.com:19302"}]
  });

  pc.ontrack = (ev) => {
    const v = document.getElementById("v");
    if (v.srcObject !== ev.streams[0]) v.srcObject = ev.streams[0];
  };

  pc.onicecandidate = (ev) => {
    if (ev.candidate && ws) {
      ws.send(JSON.stringify({
        type: "ice",
        candidate: ev.candidate.candidate,
        sdpMLineIndex: ev.candidate.sdpMLineIndex
      }));
    }
  };

  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => setStatus("ws connected, waiting for offer...");
  ws.onclose = () => setStatus("ws closed");
  ws.onerror = () => setStatus("ws error");

  ws.onmessage = async (msg) => {
    const data = JSON.parse(msg.data);

    if (data.type === "offer") {
      setStatus("offer received, creating answer...");
      await pc.setRemoteDescription({type:"offer", sdp:data.sdp});
      const answer = await pc.createAnswer();
      await pc.setLocalDescription(answer);
      ws.send(JSON.stringify({type:"answer", sdp: answer.sdp}));
      setStatus("answer sent, streaming...");
    } else if (data.type === "ice") {
      try {
        await pc.addIceCandidate({
          candidate: data.candidate,
          sdpMLineIndex: data.sdpMLineIndex
        });
      } catch (e) {
        console.log("ICE add error", e);
      }
    }
  };
}


function disconnect() {
  if (ws) { ws.close(); ws = null; }
  if (pc) { pc.close(); pc = null; }
  setStatus("disconnected");
}

function initPage() {
  connectStats(); // stats always on
}
window.addEventListener("load", initPage);



function connectStats() {
  if (wsStats && wsStats.readyState === WebSocket.OPEN) return;

  wsStats = new WebSocket(`ws://${location.host}/ws_stats`);
  wsStats.onopen = () => {
    document.getElementById("stats").textContent = "connected, waiting for first sample...";
  };
  wsStats.onclose = () => {
    // Auto-reconnect
    setTimeout(connectStats, 1000);
  };
  wsStats.onerror = () => {
    // Let close() drive reconnection
  };
  wsStats.onmessage = (msg) => {
    document.getElementById("stats").textContent = msg.data;
  };
}


document.getElementById("exp").addEventListener("input", async (ev) => {
  const v = parseInt(ev.target.value);
  document.getElementById("expv").textContent = v;
  await postJSON("/api/exposure", {us:v});
});

document.getElementById("gain").addEventListener("input", async (ev) => {
  const v = parseFloat(ev.target.value);
  document.getElementById("gainv").textContent = v.toFixed(1);
  await postJSON("/api/gain", {gain:v});
});

document.getElementById("fps").addEventListener("input", (ev) => {
  const v = parseInt(ev.target.value);
  document.getElementById("fpsv").textContent = v;
});

async function applyFps() {
  const v = parseInt(document.getElementById("fps").value);
  await postJSON("/api/fps", {fps:v});
}
</script>
</body>
</html>
"""

@app.get("/")
def root():
    return HTMLResponse(HTML_PAGE)

# ---- REST controls ----

@app.post("/api/ae")
def api_ae(payload: Dict[str, Any] = Body(...)):
    enabled = bool(payload.get("enabled", True))
    camera.set_ae(enabled)
    return JSONResponse({"ok": True, "ae_enable": enabled})

@app.post("/api/exposure")
def api_exposure(payload: Dict[str, Any] = Body(...)):
    us = int(payload.get("us", 20000))
    camera.set_exposure_us(us)
    return JSONResponse({"ok": True, "exposure_time_us": us})

@app.post("/api/gain")
def api_gain(payload: Dict[str, Any] = Body(...)):
    gain = float(payload.get("gain", 4.0))
    camera.set_gain(gain)
    return JSONResponse({"ok": True, "analogue_gain": gain})

@app.post("/api/fps")
def api_fps(payload: Dict[str, Any] = Body(...)):
    fps = int(payload.get("fps", 30))
    # Restart pipeline to apply FPS cleanly (future: renegotiate caps without restart).
    camera._glib_call(camera.restart_with_fps, fps)
    return JSONResponse({"ok": True, "fps": fps, "note": "pipeline restarted"})

@app.get("/api/stats")
def api_stats():
    return JSONResponse(camera.get_stats())

# ---- WebSocket signalling for WebRTC ----

@app.websocket("/ws")
async def ws_signalling(ws: WebSocket):
    await ws.accept()
    camera.attach_signalling_ws(ws)
    try:
        while True:
            msg = await ws.receive_text()
            data = json.loads(msg)

            if data.get("type") == "answer":
                camera.apply_answer_sdp(data["sdp"])

            elif data.get("type") == "ice":
                camera.add_ice_candidate(data["sdpMLineIndex"], data["candidate"])
    except WebSocketDisconnect:
        camera.detach_signalling_ws(ws)

@app.on_event("startup")
async def _capture_asyncio_loop():
    camera._asyncio_loop = asyncio.get_running_loop()

# ---- WebSocket live stats ----

@app.websocket("/ws_stats")
async def ws_stats(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            s = camera.get_stats()
            await ws.send_text(json.dumps(s, indent=2))
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass



