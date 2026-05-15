# DroneEye – Implementation Plan: Flaws & Solutions

> Cross-referenced against the official BGI Hackathon IT2P2 problem statement and internal architecture review.

---

## 🔴 Critical Flaws

### 1. ROS 2 Humble vs Iron Ambiguity
**Flaw:** The tech stack lists `ROS 2 (Humble/Iron)` without committing to one. These are different distros with different package compatibility. ROS 2 Iron reached EOL in November 2024.

**Solution:** Lock to **ROS 2 Humble** (LTS, supported till 2027). It has the best PX4 + Gazebo community support and most Docker base images are built on it.

---

### 2. No YOLOv8 Training Dataset or Pipeline
**Flaw:** Phase 3 says "pre-trained or fine-tuned YOLOv8" with no dataset, no training script, and no evaluation metrics. A default COCO-pretrained model will NOT detect structural cracks or rust — those classes don't exist in COCO. Judges will directly ask: *"What dataset? What's your mAP?"*

**Solution:** Add a fine-tuning step using one of the following datasets:
- **CODEBRIM** – Concrete Defect Bridge Image Dataset (cracks, spalling, efflorescence)
- **RDD2022** – Road Damage Detection Dataset (crack classification)

Run at minimum 10–20 epochs of transfer learning and report **mAP@50** as your accuracy metric.

---

### 3. Wrong ROS 2 ↔ FastAPI Bridge (`roslibpy`)
**Flaw:** `roslibpy` is a client-side Python library that connects *to* a rosbridge server — it cannot cleanly run inside FastAPI without threading hacks and race conditions.

**Solution:** Use one of these instead:
- **`rosbridge_suite`** WebSocket server + `roslibjs` on the frontend (simplest)
- **`rclpy` in a background `asyncio` thread** inside FastAPI (cleanest for pure Python)
- Avoid `roslibpy` inside the FastAPI process entirely

---

### 4. No Obstacle Detection & Avoidance
**Flaw:** The official problem statement explicitly requires *"Obstacle detection and avoidance."* The current navigation node only does blind waypoint-following — in a Gazebo world with geometry, the drone will fly into obstacles.

**Solution:** Add a laser scan / depth sensor subscriber in the navigation node:
```python
# Subscribe to /scan or /depth/points
# If min_range < threshold → pause waypoint, hover, reroute
```
Use ROS 2's `sensor_msgs/LaserScan` topic. A simple proximity halt is enough for a hackathon demo.

---

### 5. No Geo-Fencing / Safety Mechanisms
**Flaw:** The problem statement explicitly lists *"Geo-fencing and safety mechanisms"* as a required navigation feature. Your plan has no mention of this.

**Solution:** Add a bounding box constraint in the navigation node:
```python
GEO_FENCE = {"x_min": -50, "x_max": 50, "y_min": -50, "y_max": 50, "z_max": 30}
# Before publishing each waypoint, assert it falls within bounds
# If breached → trigger RETURN_TO_LAUNCH immediately
```

---

### 6. Missing FAILSAFE / EMERGENCY_LAND State
**Flaw:** The state machine only has `TAKEOFF → TRANSIT → ORBIT/SURVEY → RETURN_TO_LAUNCH`. There is no `EMERGENCY_LAND` or `FAILSAFE` state for connection loss, low battery, or critical errors.

**Solution:** Add two states to the state machine:
- `FAILSAFE` – triggered on comms loss or battery < 15% → initiates controlled descent
- `EMERGENCY_LAND` – immediate vertical descent on critical error
This signals systems maturity and takes ~10 minutes to add.

---

### 7. No Cloud / Edge Deployment Strategy
**Flaw:** The problem statement calls out *"Cloud/edge-based data processing."* The entire architecture is local Docker with no mention of how it scales or deploys to real infrastructure.

**Solution:** Add a brief architecture note:
- **Edge:** YOLOv8 inference node deployable to Jetson Nano / Raspberry Pi 5 (mark as stretch goal)
- **Cloud:** FastAPI backend containerized and deployable to AWS EC2 / GCP Cloud Run
- Mention this explicitly in your System Architecture slide

---

## 🟡 Moderate Gaps

### 8. MJPEG vs WebRTC Indecision
**Flaw:** Phase 4 lists *"MJPEG server or WebRTC"* as an OR. WebRTC introduces signaling server complexity (STUN/TURN) that will waste hours at a hackathon.

**Solution:** Commit to **MJPEG** — simple, reliable, browser-native, and judges won't care about the protocol. Drop WebRTC from the plan entirely.

---

### 9. No Thermal / Multi-Sensor Architecture Mention
**Flaw:** The problem statement specifies RGB, thermal, and LiDAR (optional) sensor support. Your plan only handles `/camera/image_raw` (RGB), making the architecture appear narrow.

**Solution:** You don't need to implement thermal, but add a note in the architecture:
> *"Sensor pipeline is designed as a pluggable ROS 2 topic interface. Thermal (`/thermal/image_raw`) and LiDAR (`/scan`) can be integrated by swapping the sensor source topic — no changes required in the detection node."*

---

### 10. No Predictive Maintenance Feature
**Flaw:** The problem statement lists *"AI-based predictive maintenance insights"* as an optional enhancement. Your plan has no mention of it, missing a scoring opportunity.

**Solution:** Add a simple trend widget on the dashboard — plot defect severity scores over multiple missions. Label it *"Predictive Maintenance Timeline."* This takes 1–2 hours to implement in the Next.js frontend using a Chart.js or Recharts component.

---

### 11. No Multi-Drone Scalability Mention
**Flaw:** Multi-drone coordination is listed as an optional enhancement in the problem statement. Not mentioning it leaves scalability points on the table.

**Solution:** Add one line to your scalability/innovation section:
> *"Architecture supports multi-agent drone coordination via ROS 2 namespacing — each drone runs as an isolated namespace (`/drone_1/`, `/drone_2/`) with a shared FastAPI aggregator."*

---

## Summary Table

| # | Flaw | Priority | Fix Effort |
|---|------|----------|------------|
| 1 | ROS 2 Humble vs Iron ambiguity | 🔴 Critical | 5 mins |
| 2 | No YOLOv8 training dataset/pipeline | 🔴 Critical | 2–3 hrs |
| 3 | Wrong `roslibpy` bridge in FastAPI | 🔴 Critical | 1 hr |
| 4 | No obstacle avoidance | 🔴 Critical | 1–2 hrs |
| 5 | No geo-fencing | 🔴 Critical | 30 mins |
| 6 | Missing FAILSAFE state | 🔴 Critical | 30 mins |
| 7 | No cloud/edge strategy | 🔴 Critical | 30 mins (docs) |
| 8 | MJPEG vs WebRTC indecision | 🟡 Moderate | 5 mins |
| 9 | No thermal/multi-sensor mention | 🟡 Moderate | 15 mins (docs) |
| 10 | No predictive maintenance feature | 🟡 Moderate | 1–2 hrs |
| 11 | No multi-drone scalability mention | 🟡 Moderate | 10 mins (docs) |
