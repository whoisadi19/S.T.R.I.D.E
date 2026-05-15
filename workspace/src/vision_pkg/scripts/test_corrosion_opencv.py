#!/usr/bin/env python3
"""Offline OpenCV corrosion/rust detection tester.

This script does not require ROS, Gazebo, YOLO, or internet access. It uses the
same style of HSV color thresholding and contour extraction as the ROS vision
node so corrosion/rust detection can be checked from a still image.
"""

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np


DEFECT_COLORS = {
    "rust/corrosion": (0, 140, 255),
}


def classify_severity(confidence):
    if confidence >= 0.8:
        return "HIGH"
    if confidence >= 0.5:
        return "MEDIUM"
    return "LOW"


def draw_label(image, text, x, y, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    thickness = 1
    y = max(y, 18)
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
    cv2.rectangle(image, (x, y - th - 8), (x + tw + 6, y + 2), color, -1)
    cv2.putText(
        image,
        text,
        (x + 3, y - 4),
        font,
        font_scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )


def touches_multiple_borders(x, y, box_w, box_h, width, height, margin=30):
    borders = 0
    if x <= margin:
        borders += 1
    if y <= margin:
        borders += 1
    if x + box_w >= width - margin:
        borders += 1
    if y + box_h >= height - margin:
        borders += 1
    return borders >= 2


def touches_bottom_foreground(x, y, box_w, box_h, width, height, margin=30):
    return y + box_h >= height - margin and y > height * 0.45


def bbox_iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    intersection = (ix2 - ix1) * (iy2 - iy1)
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    return intersection / float(area_a + area_b - intersection)


def find_beam_like_corrosion(image, hsv, rust_mask):
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=80,
        minLineLength=int(width * 0.28),
        maxLineGap=35,
    )
    if lines is None:
        return None

    beam_lines = []
    for line in lines[:, 0]:
        x1, y1, x2, y2 = map(int, line)
        length = math.hypot(x2 - x1, y2 - y1)
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        y_mid = (y1 + y2) / 2.0
        if length < width * 0.28:
            continue
        if abs(angle) > 22:
            continue
        if not (height * 0.15 < y_mid < height * 0.70):
            continue
        beam_lines.append((x1, y1, x2, y2, length))

    if not beam_lines:
        return None

    xs = []
    ys = []
    for x1, y1, x2, y2, _ in beam_lines:
        xs.extend([x1, x2])
        ys.extend([y1, y2])

    pad_x = int(width * 0.02)
    pad_y = int(height * 0.025)
    x1 = max(0, min(xs) - pad_x)
    y1 = max(0, min(ys) - pad_y)
    x2 = min(width, max(xs) + pad_x)
    y2 = min(height, max(ys) + pad_y)
    if max(xs) > width * 0.82:
        x2 = width

    box_w = x2 - x1
    box_h = y2 - y1
    if box_w < width * 0.40 or box_h < height * 0.05:
        return None
    if touches_bottom_foreground(x1, y1, box_w, box_h, width, height):
        return None

    roi_mask = rust_mask[y1:y2, x1:x2]
    rust_ratio = cv2.countNonZero(roi_mask) / float(box_w * box_h)
    if rust_ratio < 0.05:
        return None

    confidence = min(0.95, 0.60 + rust_ratio * 1.5 + min(len(beam_lines), 4) * 0.04)
    return {
        "class": "rust/corrosion",
        "confidence": round(confidence, 3),
        "bbox": [int(x1), int(y1), int(x2), int(y2)],
        "severity": classify_severity(confidence),
        "area_px": int(box_w * box_h * rust_ratio),
        "timestamp": time.time(),
    }


def detect_corrosion(image, min_area=350):
    detections = []
    annotated = image.copy()
    height, width = image.shape[:2]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Strong rust catches saturated orange/brown corrosion. Pale rust catches
    # low-saturation oxidized metal, but is only trusted for long beam-like
    # shapes later so grey floors do not dominate the result.
    strong_rust = cv2.inRange(hsv, np.array([8, 45, 45]), np.array([30, 255, 215]))
    pale_rust = cv2.inRange(hsv, np.array([8, 12, 70]), np.array([38, 120, 230]))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    beam_mask = cv2.bitwise_or(strong_rust, pale_rust)
    beam_mask = cv2.morphologyEx(beam_mask, cv2.MORPH_OPEN, kernel)
    beam_mask = cv2.morphologyEx(beam_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    beam_detection = find_beam_like_corrosion(image, hsv, beam_mask)
    if beam_detection:
        detections.append(beam_detection)

    combined_mask = strong_rust
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(
        combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        x, y, box_w, box_h = cv2.boundingRect(contour)
        box_area = box_w * box_h
        frame_area = width * height
        aspect_ratio = max(box_w, box_h) / max(1, min(box_w, box_h))

        if box_area > frame_area * 0.45:
            continue
        if touches_multiple_borders(x, y, box_w, box_h, width, height):
            continue
        if touches_bottom_foreground(x, y, box_w, box_h, width, height):
            continue

        roi_mask = combined_mask[y : y + box_h, x : x + box_w]
        roi_hsv = hsv[y : y + box_h, x : x + box_w]
        fill_ratio = cv2.countNonZero(roi_mask) / float(box_area)
        mean_saturation = cv2.mean(roi_hsv[:, :, 1], mask=roi_mask)[0]
        is_long_beam = box_w > width * 0.35 and aspect_ratio > 3.0

        if fill_ratio < 0.08:
            continue
        if mean_saturation < 55 and not is_long_beam:
            continue
        if area < 2500 and aspect_ratio < 1.8:
            continue

        confidence = min(
            0.95,
            0.42
            + (area / frame_area) * 3.5
            + fill_ratio * 0.22
            + min(mean_saturation / 255.0, 1.0) * 0.18,
        )

        defect_class = "rust/corrosion"

        detection = {
            "class": defect_class,
            "confidence": round(confidence, 3),
            "bbox": [int(x), int(y), int(x + box_w), int(y + box_h)],
            "severity": classify_severity(confidence),
            "area_px": int(area),
            "timestamp": time.time(),
        }
        if any(bbox_iou(detection["bbox"], item["bbox"]) > 0.25 for item in detections):
            continue
        detections.append(detection)

    for detection in detections:
        x, y, x2, y2 = detection["bbox"]
        defect_class = detection["class"]
        color = DEFECT_COLORS[defect_class]
        cv2.rectangle(annotated, (x, y), (x2, y2), color, 2)
        draw_label(
            annotated,
            f'{defect_class} {detection["confidence"]:.0%}',
            x,
            y,
            color,
        )

    detections.sort(key=lambda item: item["confidence"], reverse=True)
    return detections, annotated, combined_mask


def create_sample_image(path):
    image = np.full((420, 640, 3), (165, 165, 155), dtype=np.uint8)
    cv2.rectangle(image, (120, 40), (520, 380), (145, 145, 138), -1)

    rng = np.random.default_rng(7)
    for center, axes, color in [
        ((260, 165), (58, 30), (35, 95, 165)),
        ((390, 255), (78, 42), (22, 75, 130)),
        ((210, 285), (40, 22), (30, 80, 150)),
    ]:
        cv2.ellipse(image, center, axes, 0, 0, 360, color, -1)

    for _ in range(900):
        x = int(rng.integers(120, 520))
        y = int(rng.integers(40, 380))
        noise = int(rng.integers(-18, 18))
        image[y, x] = np.clip(image[y, x].astype(int) + noise, 0, 255)

    cv2.putText(
        image,
        "sample inspection panel",
        (18, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (40, 40, 40),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(str(path), image)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test corrosion/rust detection offline using OpenCV."
    )
    parser.add_argument("image", nargs="?", help="Input image path")
    parser.add_argument(
        "--output",
        default="corrosion_annotated.jpg",
        help="Annotated image output path",
    )
    parser.add_argument(
        "--mask",
        default=None,
        help="Optional binary mask output path",
    )
    parser.add_argument(
        "--min-area",
        type=int,
        default=350,
        help="Minimum detected patch area in pixels",
    )
    parser.add_argument(
        "--make-sample",
        action="store_true",
        help="Create and test a synthetic corrosion sample image",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    image_path = Path(args.image or "sample_corrosion_input.jpg")
    if args.make_sample:
        create_sample_image(image_path)

    image = cv2.imread(str(image_path))
    if image is None:
        raise SystemExit(f"Could not read image: {image_path}")

    detections, annotated, mask = detect_corrosion(image, min_area=args.min_area)

    output_path = Path(args.output)
    cv2.imwrite(str(output_path), annotated)

    if args.mask:
        cv2.imwrite(str(args.mask), mask)

    print(
        json.dumps(
            {
                "input": str(image_path),
                "output": str(output_path),
                "num_detections": len(detections),
                "detections": detections,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
