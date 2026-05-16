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
        
        # Simulate Telemetry Updates (since we don't have a full mavros stack here)
        self.timer = self.create_timer(1.0, self.update_mock_telemetry)
        self.start_time = time.time()

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

    def update_mock_telemetry(self):
        global telemetry_data
        elapsed = time.time() - self.start_time
        
        # Simulate Battery Drain
        telemetry_data["battery"] = max(0, 100 - int(elapsed / 10))
        
        # Simulate Altitude and Phase
        if elapsed < 5:
            telemetry_data["phase"] = "PRE_FLIGHT"
            telemetry_data["altitude"] = 1.0
            telemetry_data["gps"] = "34.0522 N, 118.2437 W"
        elif elapsed < 15:
            telemetry_data["phase"] = "TAKEOFF"
            telemetry_data["altitude"] = min(10.0, 1.0 + (elapsed - 5) * 1.5)
            telemetry_data["speed"] = 1.5
        else:
            telemetry_data["phase"] = "INSPECT_TOWER"
            telemetry_data["altitude"] = 10.0 + np.sin(elapsed / 2.0) * 0.5
            telemetry_data["speed"] = 0.5

        # Simulate Signal Fluctuation
        telemetry_data["signal"] = 90 + int(np.random.normal(0, 5))

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
