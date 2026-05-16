import asyncio
import json
import time
import cv2
import numpy as np
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

app = FastAPI(title="S.T.R.I.D.E. Backend")

# Allow CORS for Next.js
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global variables for ROS 2 data
latest_frame = None
latest_defects = []
telemetry_data = {
    "battery": 100,
    "altitude": 0.0,
    "phase": "PRE_FLIGHT",
    "speed": 0.0,
    "gps": "ACQUIRING...",
    "signal": 100,
}

class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('fastapi_bridge_node')
        self.image_sub = self.create_subscription(
            CompressedImage, '/vision/annotated_image', self.image_callback, 10)
        self.defect_sub = self.create_subscription(
            String, '/vision/defects', self.defect_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/mission/status', self.status_callback, 10)
        
    def image_callback(self, msg):
        global latest_frame
        latest_frame = msg.data

    def defect_callback(self, msg):
        global latest_defects
        try:
            data = json.loads(msg.data)
            latest_defects = data.get('defects', [])
        except Exception as e:
            self.get_logger().error(f"Defect parse error: {e}")

    def status_callback(self, msg):
        global telemetry_data
        try:
            data = json.loads(msg.data)
            telemetry_data["battery"] = data.get("battery_percent", 100)
            telemetry_data["phase"] = data.get("phase", "UNKNOWN")
            pos = data.get("position", {"z": 0.0, "x": 0.0, "y": 0.0})
            telemetry_data["altitude"] = pos.get("z", 0.0)
            telemetry_data["gps"] = f"34.0522 N, 118.2437 W | x:{pos.get('x',0.0):.1f} y:{pos.get('y',0.0):.1f}"
        except Exception as e:
            pass

# --- Background ROS 2 Thread ---
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

import threading
ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

# --- FastAPI Endpoints ---

def generate_mjpeg():
    """Generator for MJPEG stream from ROS 2 CompressedImage"""
    while True:
        if latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        else:
            # Fallback black frame if no ROS data
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "AWAITING VIDEO STREAM...", (180, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            _, jpeg = cv2.imencode('.jpg', blank)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.05)  # 20 FPS

@app.get("/video_feed")
def video_feed():
    """HTTP endpoint for the MJPEG stream."""
    return StreamingResponse(generate_mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    """WebSocket endpoint for live telemetry and defects."""
    await websocket.accept()
    try:
        while True:
            payload = {
                "telemetry": telemetry_data,
                "defects": latest_defects,
                "timestamp": time.time()
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.5) # 2 Hz updates
    except Exception as e:
        print(f"WebSocket disconnected: {e}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
