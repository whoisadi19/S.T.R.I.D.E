"""
S.T.R.I.D.E. FastAPI Backend — Real-Time ROS 2 Bridge

Subscribes to actual ROS 2 topics from the simulation:
  /drone/telemetry       — JSON telemetry from kinematics_node (2Hz)
  /drone/battery         — JSON battery status from kinematics_node (20Hz)
  /mission/status        — JSON mission phase from navigation_node
  /vision/annotated_image — CompressedImage JPEG from defect_detection_node
  /vision/defects        — JSON defect list from defect_detection_node
  /drone_camera/image_raw — Raw camera Image (fallback if vision node not running)

Serves:
  GET  /video_feed      — MJPEG stream for the dashboard
  WS   /ws/telemetry    — WebSocket for live telemetry + defects
"""

import asyncio
import json
import time
import threading
import cv2
import numpy as np
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

app = FastAPI(title="S.T.R.I.D.E. Backend")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Shared State ─────────────────────────────────────────────────────
latest_jpeg = None          # bytes: latest JPEG frame for MJPEG stream
latest_defects = []         # list of defect dicts
telemetry_data = {
    "battery": 100.0,
    "altitude": 0.0,
    "phase": "AWAITING_LINK",
    "speed": 0.0,
    "gps": "-- , --",
    "signal": 95,
    "heading": 0.0,
    "flight_time": 0.0,
    "distance": 0.0,
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
}


class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('fastapi_bridge_node')
        self.get_logger().info('S.T.R.I.D.E. Backend Bridge starting...')

        # ── Vision: annotated frames (CompressedImage JPEG) ──────────
        self.create_subscription(
            CompressedImage, '/vision/annotated_image',
            self.annotated_image_cb, 10)

        # ── Vision fallback: raw camera feed (if vision node is down) ──
        self.create_subscription(
            Image, '/drone_camera/image_raw',
            self.raw_image_cb, 10)

        # ── Defects ──────────────────────────────────────────────────
        self.create_subscription(
            String, '/vision/defects',
            self.defect_cb, 10)

        # ── Telemetry from kinematics_node (2Hz, comprehensive) ──────
        self.create_subscription(
            String, '/drone/telemetry',
            self.telemetry_cb, 10)

        # ── Battery from kinematics_node (20Hz) ──────────────────────
        self.create_subscription(
            String, '/drone/battery',
            self.battery_cb, 10)

        # ── Mission status from navigation_node ──────────────────────
        self.create_subscription(
            String, '/mission/status',
            self.mission_cb, 10)

        self.has_annotated = False  # flag: prefer annotated over raw
        self.get_logger().info('Subscribed to all ROS 2 topics.')

    # ── Callbacks ────────────────────────────────────────────────────

    def annotated_image_cb(self, msg):
        global latest_jpeg
        self.has_annotated = True
        latest_jpeg = bytes(msg.data)

    def raw_image_cb(self, msg):
        global latest_jpeg
        if self.has_annotated:
            return  # prefer annotated frames when available
        try:
            # Manual conversion: ROS Image → numpy → JPEG (no cv_bridge needed)
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            # ROS uses RGB, OpenCV uses BGR
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            latest_jpeg = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Raw image conversion error: {e}',
                                   throttle_duration_sec=5.0)

    def defect_cb(self, msg):
        global latest_defects
        try:
            data = json.loads(msg.data)
            latest_defects = data.get('defects', [])
        except Exception:
            pass

    def telemetry_cb(self, msg):
        global telemetry_data
        try:
            data = json.loads(msg.data)
            pos = data.get('position', {})
            vel = data.get('velocity', {})
            telemetry_data["altitude"] = data.get("altitude", 0.0)
            telemetry_data["speed"] = vel.get("speed", 0.0)
            telemetry_data["heading"] = data.get("heading_deg", 0.0)
            telemetry_data["flight_time"] = data.get("flight_time_sec", 0.0)
            telemetry_data["distance"] = data.get("total_distance_m", 0.0)
            telemetry_data["position"] = pos
            telemetry_data["gps"] = (
                f"x:{pos.get('x',0.0):.1f}  y:{pos.get('y',0.0):.1f}  "
                f"z:{pos.get('z',0.0):.1f}")
        except Exception:
            pass

    def battery_cb(self, msg):
        global telemetry_data
        try:
            data = json.loads(msg.data)
            telemetry_data["battery"] = data.get("percentage", 100.0)
            telemetry_data["signal"] = 95  # simulated constant
        except Exception:
            pass

    def mission_cb(self, msg):
        global telemetry_data
        try:
            data = json.loads(msg.data)
            telemetry_data["phase"] = data.get("phase", "UNKNOWN")
        except Exception:
            pass


# ── ROS 2 Spin Thread ────────────────────────────────────────────────
def ros_spin():
    rclpy.init()
    node = RosBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()


# ── MJPEG Video Stream ──────────────────────────────────────────────
def generate_mjpeg():
    while True:
        if latest_jpeg is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   latest_jpeg + b'\r\n')
        else:
            # Black placeholder frame
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "AWAITING VIDEO STREAM...", (140, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 80), 2)
            _, jpeg = cv2.imencode('.jpg', blank)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   jpeg.tobytes() + b'\r\n')
        time.sleep(0.05)  # ~20 FPS


@app.get("/video_feed")
def video_feed():
    return StreamingResponse(
        generate_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            payload = {
                "telemetry": telemetry_data,
                "defects": latest_defects,
                "timestamp": time.time(),
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.5)  # 2 Hz
    except Exception:
        pass


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
