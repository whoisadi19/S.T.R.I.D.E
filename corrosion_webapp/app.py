from pathlib import Path
from uuid import uuid4

import cv2
from flask import Flask, jsonify, render_template, request, send_from_directory
from werkzeug.utils import secure_filename

import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
DETECTOR_DIR = ROOT_DIR / "workspace" / "src" / "vision_pkg" / "scripts"
sys.path.insert(0, str(DETECTOR_DIR))

from test_corrosion_opencv import detect_corrosion  # noqa: E402


APP_DIR = Path(__file__).resolve().parent
UPLOAD_DIR = APP_DIR / "uploads"
RESULT_DIR = APP_DIR / "results"
ALLOWED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

UPLOAD_DIR.mkdir(exist_ok=True)
RESULT_DIR.mkdir(exist_ok=True)

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = 12 * 1024 * 1024


def allowed_file(filename):
    return Path(filename).suffix.lower() in ALLOWED_EXTENSIONS


@app.route("/", methods=["GET", "POST"])
def index():
    error = None
    result = None

    if request.method == "POST":
        uploaded = request.files.get("image")
        if not uploaded or uploaded.filename == "":
            error = "Please choose an image first."
        elif not allowed_file(uploaded.filename):
            error = "Upload a JPG, PNG, BMP, or WEBP image."
        else:
            original_name = secure_filename(uploaded.filename)
            suffix = Path(original_name).suffix.lower()
            image_id = uuid4().hex
            upload_name = f"{image_id}{suffix}"
            result_name = f"{image_id}_annotated.jpg"

            input_path = UPLOAD_DIR / upload_name
            output_path = RESULT_DIR / result_name
            uploaded.save(input_path)

            image = cv2.imread(str(input_path))
            if image is None:
                error = "OpenCV could not read that image."
            else:
                detections, annotated, _ = detect_corrosion(image)
                cv2.imwrite(str(output_path), annotated)
                result = {
                    "input_url": f"/uploads/{upload_name}",
                    "output_url": f"/results/{result_name}",
                    "detections": detections,
                    "count": len(detections),
                }

    return render_template("index.html", error=error, result=result)


@app.route("/detect", methods=["POST"])
def detect():
    uploaded = request.files.get("image")
    if not uploaded or uploaded.filename == "":
        return jsonify({"error": "Please choose an image first."}), 400
    if not allowed_file(uploaded.filename):
        return jsonify({"error": "Upload a JPG, PNG, BMP, or WEBP image."}), 400

    original_name = secure_filename(uploaded.filename)
    suffix = Path(original_name).suffix.lower()
    image_id = uuid4().hex
    upload_name = f"{image_id}{suffix}"
    result_name = f"{image_id}_annotated.jpg"

    input_path = UPLOAD_DIR / upload_name
    output_path = RESULT_DIR / result_name
    uploaded.save(input_path)

    image = cv2.imread(str(input_path))
    if image is None:
        return jsonify({"error": "OpenCV could not read that image."}), 400

    detections, annotated, _ = detect_corrosion(image)
    cv2.imwrite(str(output_path), annotated)
    return jsonify(
        {
            "input_url": f"/uploads/{upload_name}",
            "output_url": f"/results/{result_name}",
            "detections": detections,
            "count": len(detections),
        }
    )


@app.route("/uploads/<path:filename>")
def uploaded_file(filename):
    return send_from_directory(UPLOAD_DIR, filename)


@app.route("/results/<path:filename>")
def result_file(filename):
    return send_from_directory(RESULT_DIR, filename)


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=False, use_reloader=False)
