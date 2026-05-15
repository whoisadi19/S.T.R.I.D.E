# Autonomous Drone-Based Infrastructure Inspection System

This project aims to build an end-to-end simulated autonomous drone inspection system. The system will use ROS 2 and Gazebo for simulation, YOLOv8 for defect detection, FastAPI for the backend, and a Next.js frontend dashboard to view real-time data and generate reports.

## Proposed Tech Stack

*   **Simulation:** ROS 2 Humble (LTS) + Gazebo (Setup via Docker for Windows compatibility). *Note: Currently using a Kinematic Prototype (SDF + kinematics_node) for rapid prototyping, with the option to upgrade to full PX4 SITL later.*
*   **Navigation:** Custom ROS 2 Python node with offboard waypoint navigation, obstacle avoidance, and geo-fencing
*   **Computer Vision:** YOLOv8 (via Ultralytics) integrated as a ROS 2 node, fine-tuned on custom datasets
*   **Backend:** FastAPI (Python) interacting with ROS 2 via `rclpy` in a background `asyncio` thread
*   **Frontend:** Next.js + TailwindCSS + shadcn/ui (Premium Dark Dashboard)
*   **Reporting:** Python (ReportLab)

## Architecture & Phases

### Phase 1: Simulation Foundation
- Set up a ROS 2 workspace in the `autonomous_drone_inspection` directory.
- Configure a Dockerfile/Docker Compose setup to easily run ROS 2 and Gazebo on Windows (this abstracts away the complex Gazebo installation).
- Create a custom Gazebo world containing the target infrastructure (e.g., a simple tower or bridge).

### Phase 2: Autonomous Navigation
- **Gazebo Drone Model**: Creating a `simple_drone` SDF model equipped with camera and LiDAR sensors, and a `kinematics_node.py` to translate `/cmd_vel` into Gazebo `/set_entity_state` updates.
- `drone_control_pkg/navigation_node.py`: A ROS 2 node to handle offboard control, sending a sequence of waypoints to orbit/survey the infrastructure automatically.
- **Obstacle Avoidance:** Subscribes to `/scan` or `/depth/points`. If `min_range < threshold`, it pauses waypoint execution, hovers, and reroutes.
- **Geo-Fencing:** Implements safety boundaries (e.g., `x_min`, `x_max`, `z_max`). Breaching bounds triggers an immediate `RETURN_TO_LAUNCH`.
- **State Machine Architecture**: Implement a robust state machine (`TAKEOFF` -> `TRANSIT` -> `ORBIT/SURVEY` -> `RETURN_TO_LAUNCH` -> `FAILSAFE` -> `EMERGENCY_LAND`) to manage flight logic, handle comms loss (`FAILSAFE`), and critical errors (`EMERGENCY_LAND`).

### Phase 3: Vision & Defect Detection
- `vision_pkg/defect_detection_node.py`: Subscribes to the simulated drone camera feed (`/camera/image_raw`).
- **YOLOv8 Fine-Tuning**: Integrates a YOLOv8 model fine-tuned on datasets like **CODEBRIM** or **RDD2022** for 10-20 epochs to properly detect structural cracks and rust (evaluated using mAP@50). 
- **Streaming Optimization**: Instead of pushing raw arrays to the web frontend, this node compresses the annotated frames to JPEG and publishes them, while also publishing defect metadata (coordinates, severity) separately.
- **Multi-Sensor Architecture**: The sensor pipeline is designed as a pluggable ROS 2 topic interface. Thermal (`/thermal/image_raw`) and LiDAR (`/scan`) can be integrated by swapping the sensor source topic—no changes required in the detection node.

### Phase 4: Backend & Dashboard
- `backend/`: FastAPI application to serve telemetry, alerts, and report generation endpoints. Hosts a lightweight **MJPEG** server for the compressed video feed, and a WebSocket server for telemetry and defect metadata.
- `frontend/`: Next.js web application featuring a live feed, telemetry dashboard, mission control interface, and a **Predictive Maintenance Timeline** (plotting defect severity scores over multiple missions). This will have a premium "Terminal Dark" aesthetic using `shadcn/ui`.

### Phase 5: Reporting
- `backend/services/report_generator.py`: Gathers mission data and generates a structured PDF report summarizing findings, severity, and annotated images.

## Setup Strategy for Gazebo on Windows
Since running Gazebo natively on Windows can be tricky, we will create a `docker-compose.yml` and `Dockerfile` that packages ROS 2, Gazebo, and the PX4 simulator. This way, all you have to do is run `docker compose up`, and the simulation environment will spin up automatically.

## Deployment & Scalability

### The "Sim-to-Real" Flex (Edge & Cloud)
To demonstrate hardware readiness, we will structure the architecture to support a hybrid deployment approach. 
- **Edge:** The heavy ROS/Gazebo simulation runs in Docker on the host machine, while the `defect_detection_node.py` (YOLOv8 pipeline) can be deployed onto an actual edge compute device (like a Jetson Nano or Raspberry Pi 5) that processes the network stream from the simulation.
- **Cloud:** The FastAPI backend and Next.js frontend are containerized and deployable to AWS EC2 or GCP Cloud Run.

### Multi-Drone Coordination
The architecture supports multi-agent drone coordination via ROS 2 namespacing. Each drone runs as an isolated namespace (e.g., `/drone_1/`, `/drone_2/`) with a shared FastAPI aggregator handling fleet telemetry.
