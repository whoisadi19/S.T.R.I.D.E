# Project Work Log: Autonomous Drone-Based Infrastructure Inspection System

This document tracks all steps, errors, approaches, and important decisions made throughout the project.

## Initial Setup & Planning

### Approach
- We decided to use a **phased approach** to build the system:
  1. **Simulation Foundation:** Set up a ROS 2 + Gazebo environment via Docker to abstract away complex installations on Windows.
  2. **Autonomous Navigation:** Create a custom ROS 2 Python node for waypoint-based offboard control.
  3. **Vision & Defect Detection:** Integrate YOLOv8 as a ROS 2 node to process the drone's camera feed and detect defects (like cracks/rust) in real-time.
  4. **Backend & Dashboard:** Build a FastAPI backend and a premium Next.js frontend to monitor telemetry and live video.
  5. **Reporting:** Generate automated PDF reports with mission data and annotated images.

### Steps Taken
- Created `implementation.md` to document the 5-phase approach.
- Created `docker-compose.yml` to define the simulation environment container (`ros2_gazebo`). It maps ports `8080` (NoVNC), `8000` (Backend), and `3000` (Frontend).
- Created `Dockerfile` based on `osrf/ros:humble-desktop`, installing `gazebo-ros-pkgs`, `x11vnc`, `novnc`, and `fluxbox` for a web-based GUI.
- Created `start.sh` script to initialize the virtual framebuffer (Xvfb) and NoVNC server.
- Initialized a ROS 2 workspace at `workspace/src/drone_simulation/`.
- Created boilerplate ROS 2 package files: `package.xml` and `CMakeLists.txt`.
- Created a basic Gazebo world file (`workspace/src/drone_simulation/worlds/inspection.world`) featuring a simple tower structure to act as the inspection target.
- Created a ROS 2 launch file (`workspace/src/drone_simulation/launch/simulation.launch.py`) to start Gazebo with the custom world.

### Phase 1 Execution & Verification
- Started Docker daemon on Windows and ran `docker compose up --build -d` to build the `osrf/ros:humble-desktop` based container.
- Verified NoVNC interface was accessible via `http://localhost:8080`.
- Executed an interactive bash session into the container (`docker exec -it drone_simulation bash`).
- Sourced the ROS 2 Humble environment, built the `drone_simulation` package via `colcon build`, and successfully launched Gazebo with the custom inspection tower world.

### Architecture Revision (v2)
- Analyzed `Flaws_and_Solutions.md` and applied 11 critical/moderate fixes to the architecture plan.
- **Key Fixes:** Locked to ROS 2 Humble (LTS), added YOLOv8 fine-tuning plan (CODEBRIM/RDD2022), replaced `roslibpy` with pure `rclpy` + `asyncio`, added obstacle avoidance & geo-fencing, added `FAILSAFE` & `EMERGENCY_LAND` states, clarified Cloud/Edge strategy, committed to MJPEG streaming, added multi-sensor integration notes, added predictive maintenance, and added multi-drone scalability.
- Renamed the updated plan from `implementation.md` to `implementationv2.md`.

### Errors Encountered
- **Error:** Attempted to run `docker compose up --build -d` to start the simulation environment.
  - **Log:** `docker : The term 'docker' is not recognized as the name of a cmdlet, function, script file, or operable program.`
  - **Resolution:** The user's machine did not have Docker installed. Checked the system architecture (`ProcessArchitecture` returned `X64`). Advised the user to download and install the **AMD64** version of Docker Desktop for Windows.
- **Error:** Attempted to run `colcon build` inside a new `docker exec` bash session.
  - **Log:** `CMake Error at CMakeLists.txt:8 (find_package): Could not find a package configuration file provided by "ament_cmake"`
  - **Resolution:** A new interactive bash session does not inherit the environment variables from the `start.sh` entrypoint. Advised user to run `source /opt/ros/humble/setup.bash` before running `colcon build`.

### Current Status
- **Phase 1 Complete:** Docker simulation container built successfully and NoVNC interface is running locally on port 8080.
- Addressed all architectural flaws and updated the project plan to `implementationv2.md`. 
- Ready to move to Phase 2 (ROS Nodes) or Phase 4 (Backend/Frontend).
