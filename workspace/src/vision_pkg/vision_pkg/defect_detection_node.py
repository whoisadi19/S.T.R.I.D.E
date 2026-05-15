"""
Defect Detection Node — Phase 3 of Autonomous Drone Inspection System

Subscribes to the drone's camera feed, runs YOLOv8 inference for structural
defect detection (cracks, rust, corrosion, spalling), and publishes:
  - /vision/annotated_image  (sensor_msgs/CompressedImage) — JPEG annotated frames
  - /vision/defects           (std_msgs/String)             — JSON defect metadata

In simulation mode (no real defects on the Gazebo cylinder), the node uses
OpenCV edge/texture analysis to simulate defect detection for demo purposes.
When a real YOLOv8 model trained on CODEBRIM/RDD2022 is available, set
use_yolo=True via ROS param.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time

# Try importing ultralytics (YOLOv8) — graceful fallback if not installed
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class DefectDetectionNode(Node):
    def __init__(self):
        super().__init__('defect_detection_node')
        self.get_logger().info('Defect Detection Node starting...')

        # --- Parameters ---
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('yolo_model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('inference_rate', 5.0)  # Hz — process every Nth frame

        self.use_yolo = self.get_parameter('use_yolo').value
        self.model_path = self.get_parameter('yolo_model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.inference_rate = self.get_parameter('inference_rate').value

        # --- YOLOv8 Model ---
        self.model = None
        if self.use_yolo and YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'YOLOv8 model loaded: {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.model = None
        elif self.use_yolo and not YOLO_AVAILABLE:
            self.get_logger().warn('use_yolo=True but ultralytics not installed. '
                                   'Falling back to CV-based detection.')

        # --- CV Bridge ---
        self.bridge = CvBridge()

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # --- Publishers ---
        self.annotated_pub = self.create_publisher(
            CompressedImage, '/vision/annotated_image', 10)
        self.defect_pub = self.create_publisher(
            String, '/vision/defects', 10)

        # --- Rate limiting ---
        self.last_inference_time = 0.0
        self.inference_interval = 1.0 / self.inference_rate

        # --- Mission stats ---
        self.total_frames = 0
        self.total_defects_found = 0
        self.mission_defects = []  # accumulate all defects for reporting

        # --- Defect class mapping (for simulation mode) ---
        self.defect_classes = ['crack', 'rust', 'corrosion', 'spalling', 'delamination']
        self.defect_colors = {
            'crack': (0, 0, 255),       # Red
            'rust': (0, 140, 255),      # Orange
            'corrosion': (0, 255, 255), # Yellow
            'spalling': (255, 0, 255),  # Magenta
            'delamination': (255, 100, 0)  # Blue-ish
        }

        self.get_logger().info(
            f'Defect Detection ready | Mode: {"YOLOv8" if self.model else "CV-Simulation"} '
            f'| Rate: {self.inference_rate}Hz | Conf: {self.conf_threshold}')

    def image_callback(self, msg):
        """Process incoming camera frames."""
        # Rate limiting — skip frames to save CPU
        now = time.time()
        if now - self.last_inference_time < self.inference_interval:
            return
        self.last_inference_time = now
        self.total_frames += 1

        # Convert ROS Image → OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Run detection
        if self.model:
            detections, annotated = self._detect_yolo(cv_image)
        else:
            detections, annotated = self._detect_cv_simulation(cv_image)

        # Publish annotated image (JPEG compressed)
        self._publish_annotated(annotated)

        # Publish defect metadata
        if detections:
            self._publish_defects(detections)
            self.total_defects_found += len(detections)
            self.mission_defects.extend(detections)

        # Periodic stats
        if self.total_frames % 50 == 0:
            self.get_logger().info(
                f'Stats: {self.total_frames} frames processed, '
                f'{self.total_defects_found} total defects detected')

    def _detect_yolo(self, image):
        """Run YOLOv8 inference on the image."""
        results = self.model(image, conf=self.conf_threshold, verbose=False)
        detections = []
        annotated = image.copy()

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = result.names.get(cls_id, f'class_{cls_id}')

                # Map to defect categories if using a fine-tuned model
                defect = {
                    'class': cls_name,
                    'confidence': round(conf, 3),
                    'bbox': [x1, y1, x2, y2],
                    'severity': self._classify_severity(conf),
                    'timestamp': time.time(),
                    'frame_id': self.total_frames
                }
                detections.append(defect)

                # Neon Color Coding based on Severity
                color = self._get_severity_color(defect['severity'])
                
                # Draw Cyber-Physical Brackets instead of basic rectangle
                self._draw_hud_bracket(annotated, x1, y1, x2, y2, color)
                label = f'{cls_name.upper()} {conf:.0%}'
                self._draw_label(annotated, label, x1, y1, color)

        return detections, annotated

    def _detect_cv_simulation(self, image):
        """
        Simulation-mode defect detection using OpenCV.
        
        Uses edge detection and texture analysis to find "anomalies" on the
        tower surface. This simulates what a real defect detector would find
        and allows the full pipeline to be tested end-to-end.
        """
        detections = []
        annotated = image.copy()
        h, w = image.shape[:2]

        # Convert to grayscale and apply processing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # --- Method 1: Edge-based crack detection ---
        edges = cv2.Canny(blurred, 50, 150)
        # Dilate edges to connect nearby fragments
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges_dilated = cv2.dilate(edges, kernel, iterations=1)

        # Find contours from edges
        contours, _ = cv2.findContours(
            edges_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            # Filter: only consider contours of meaningful size
            if 200 < area < (w * h * 0.3):
                x, y, cw, ch = cv2.boundingRect(contour)
                aspect_ratio = max(cw, ch) / (min(cw, ch) + 1e-5)

                # Long thin contours → cracks
                if aspect_ratio > 3.0 and area > 300:
                    conf = min(0.95, 0.4 + (aspect_ratio / 20.0))
                    defect_type = 'crack'
                # Blobby contours → corrosion/spalling
                elif aspect_ratio < 2.5 and area > 500:
                    conf = min(0.90, 0.35 + (area / (w * h)))
                    defect_type = 'corrosion' if area < 2000 else 'spalling'
                else:
                    continue

                defect = {
                    'class': defect_type,
                    'confidence': round(conf, 3),
                    'bbox': [x, y, x + cw, y + ch],
                    'severity': self._classify_severity(conf),
                    'timestamp': time.time(),
                    'frame_id': self.total_frames
                }
                detections.append(defect)

                color = self._get_severity_color(defect['severity'])
                self._draw_hud_bracket(annotated, x, y, x + cw, y + ch, color)
                label = f'{defect_type.upper()} {conf:.0%}'
                self._draw_label(annotated, label, x, y, color)

        # --- Method 2: Color-based rust detection ---
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Rust is typically orange-brown: H=5-25, S>50, V>50
        lower_rust = np.array([5, 50, 50])
        upper_rust = np.array([25, 255, 255])
        rust_mask = cv2.inRange(hsv, lower_rust, upper_rust)

        rust_contours, _ = cv2.findContours(
            rust_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in rust_contours:
            area = cv2.contourArea(contour)
            if area > 400:
                x, y, cw, ch = cv2.boundingRect(contour)
                conf = min(0.92, 0.5 + (area / (w * h * 2)))
                defect = {
                    'class': 'rust',
                    'confidence': round(conf, 3),
                    'bbox': [x, y, x + cw, y + ch],
                    'severity': self._classify_severity(conf),
                    'timestamp': time.time(),
                    'frame_id': self.total_frames
                }
                detections.append(defect)

                color = self._get_severity_color(defect['severity'])
                self._draw_hud_bracket(annotated, x, y, x + cw, y + ch, color)
                label = f'RUST {conf:.0%}'
                self._draw_label(annotated, label, x, y, color)

        # --- HUD overlay ---
        self._draw_hud(annotated, len(detections))

        # Limit detections per frame to avoid spam
        detections = detections[:10]
        return detections, annotated

    def _classify_severity(self, confidence):
        """Map confidence score to a severity level."""
        if confidence >= 0.8:
            return 'CRITICAL'
        elif confidence >= 0.5:
            return 'WARNING'
        else:
            return 'LOW'

    def _get_severity_color(self, severity):
        """Return BGR color for severity."""
        if severity == 'CRITICAL':
            return (60, 0, 255)  # Neon Red (BGR)
        elif severity == 'WARNING':
            return (0, 230, 255) # Neon Yellow (BGR)
        else:
            return (0, 255, 0)   # Neon Green (BGR)

    def _draw_hud_bracket(self, image, x1, y1, x2, y2, color, thickness=2, length=15):
        """Draws targeting brackets instead of a full bounding box."""
        # Top-left corner
        cv2.line(image, (x1, y1), (x1 + length, y1), color, thickness)
        cv2.line(image, (x1, y1), (x1, y1 + length), color, thickness)
        # Top-right corner
        cv2.line(image, (x2, y1), (x2 - length, y1), color, thickness)
        cv2.line(image, (x2, y1), (x2, y1 + length), color, thickness)
        # Bottom-left corner
        cv2.line(image, (x1, y2), (x1 + length, y2), color, thickness)
        cv2.line(image, (x1, y2), (x1, y2 - length), color, thickness)
        # Bottom-right corner
        cv2.line(image, (x2, y2), (x2 - length, y2), color, thickness)
        cv2.line(image, (x2, y2), (x2, y2 - length), color, thickness)

    def _draw_label(self, image, text, x, y, color):
        """Draw a label with background on the image."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
        # Background rectangle
        cv2.rectangle(image, (x, y - th - 6), (x + tw + 4, y), color, -1)
        # Text
        cv2.putText(image, text, (x + 2, y - 4), cv2.FONT_HERSHEY_PLAIN, 1.0,
                    (0, 0, 0), 1, cv2.LINE_AA)

    def _draw_hud(self, image, num_detections):
        """Draw a cyber-physical heads-up display overlay on the frame."""
        h, w = image.shape[:2]

        # 1. Semi-transparent top & bottom bars
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 40), (20, 20, 20), -1)
        cv2.rectangle(overlay, (0, h - 30), (w, h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

        # 2. Central Scanning Crosshair
        cx, cy = w // 2, h // 2
        cv2.line(image, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 1)
        cv2.line(image, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 1)
        cv2.circle(image, (cx, cy), 15, (0, 255, 0), 1)
        
        # Add dynamic scanning tick marks based on frame counter
        tick_offset = (self.total_frames * 2) % 20
        cv2.line(image, (cx - 30 + tick_offset, cy - 10), (cx - 30 + tick_offset, cy + 10), (0, 200, 0), 1)
        cv2.line(image, (cx + 30 - tick_offset, cy - 10), (cx + 30 - tick_offset, cy + 10), (0, 200, 0), 1)

        # 3. Top HUD text
        font = cv2.FONT_HERSHEY_SIMPLEX
        timestamp = time.strftime('%H:%M:%S')
        hud_text = f'S.T.R.I.D.E. AUTO-INSPECTION | SYS: ONLINE | FPS: {self.inference_rate}'
        cv2.putText(image, hud_text, (10, 25), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        # 4. Bottom HUD text (Coordinates & Timestamps)
        coord_text = f'LAT: 34.0522 N | LON: 118.2437 W | ALT: 12.4m | TIME: {timestamp}'
        cv2.putText(image, coord_text, (10, h - 10), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

        # 5. Flashing Critical Warning
        if num_detections > 0:
            # Flash red based on time
            if int(time.time() * 4) % 2 == 0:
                warning_text = 'STRUCTURAL RISK DETECTED'
                (tw, th), _ = cv2.getTextSize(warning_text, font, 0.7, 2)
                cv2.putText(image, warning_text, (w // 2 - tw // 2, h - 50), font, 0.7, (60, 0, 255), 2, cv2.LINE_AA)
        
        # Top-right Status indicator
        status_color = (60, 0, 255) if num_detections > 0 else (0, 255, 0)
        cv2.putText(image, "AI", (w - 45, 25), font, 0.5, status_color, 1, cv2.LINE_AA)
        cv2.circle(image, (w - 20, 21), 6, status_color, -1)

    def _publish_annotated(self, image):
        """Publish annotated frame as JPEG CompressedImage."""
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        _, jpeg_data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        msg.data = jpeg_data.tobytes()
        self.annotated_pub.publish(msg)

    def _publish_defects(self, detections):
        """Publish defect metadata as JSON string."""
        msg = String()
        payload = {
            'frame_id': self.total_frames,
            'timestamp': time.time(),
            'num_defects': len(detections),
            'defects': detections
        }
        msg.data = json.dumps(payload)
        self.defect_pub.publish(msg)

        # Log significant detections
        for d in detections:
            if d['severity'] in ('HIGH', 'MEDIUM'):
                self.get_logger().info(
                    f'⚠ {d["severity"]} {d["class"]} detected | '
                    f'conf={d["confidence"]:.0%} | bbox={d["bbox"]}')


def main(args=None):
    rclpy.init(args=args)
    node = DefectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
