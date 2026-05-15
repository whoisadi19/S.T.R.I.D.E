# User Guide: Starting the Simulation Environment

This guide provides step-by-step instructions on how to start the ROS 2 and Gazebo simulation environment we built in Phase 1. 

## 1. Start the Docker Container

The entire environment (ROS 2 Humble, Gazebo, and the NoVNC web interface) is containerized using Docker. 

Open your Windows PowerShell (or Command Prompt) and navigate to the project directory:
```powershell
cd C:\Users\User\OneDrive\Desktop\codinggg\Projects\autonomous_drone_inspection
```

Start the container in the background using Docker Compose:
```powershell
docker compose up -d
```
*(Note: Since the image is already built, this should only take a few seconds. If you ever need to rebuild the image after changing the `Dockerfile`, append `--build` to the command).*

## 2. Access the Visual Interface (NoVNC)

Because Gazebo requires a graphical interface, we run a virtual X11 display inside the container and broadcast it via NoVNC.

1. Open your web browser (Chrome, Edge, Firefox).
2. Navigate to: **[http://localhost:8080](http://localhost:8080)**
3. You should see a blank Linux desktop environment (running the lightweight Fluxbox window manager).

## 3. Launching Gazebo and the ROS 2 Workspace

You can launch Gazebo from within the NoVNC interface or from your host Windows terminal.

### Option A: Running from Host Terminal (Recommended)
This is often faster and allows you to easily copy/paste commands.

1. Open a new PowerShell window on your Windows machine.
2. Enter the running Docker container:
   ```powershell
   docker exec -it drone_simulation bash
   ```
3. Once inside the container (`root@...:/workspace#`), source the ROS 2 environment and export the display variable so Gazebo knows to render on NoVNC:
   ```bash
   source /opt/ros/humble/setup.bash
   export DISPLAY=:0
   ```
4. Build the workspace and source the local setup file:
   ```bash
   colcon build
   source install/setup.bash
   ```
5. Launch the simulation world:
   ```bash
   ros2 launch drone_simulation simulation.launch.py
   ```
6. Switch back to your browser window (`http://localhost:8080`), and Gazebo will be running with the inspection tower!

### Option B: Running from NoVNC Terminal
If you prefer doing everything directly inside the browser window:

1. Go to `http://localhost:8080`.
2. **Right-click** anywhere on the empty desktop background.
3. Select **xterm** or **Terminal** from the context menu to open a terminal.
4. Run the build and launch commands:
   ```bash
   cd /workspace
   colcon build
   source install/setup.bash
   ros2 launch drone_simulation simulation.launch.py
   ```
*(Note: The NoVNC terminal spawned by Fluxbox already inherits the `DISPLAY=:0` and basic ROS 2 environment variables from `start.sh`, so manual sourcing of `/opt/ros/humble/setup.bash` is usually not required unless you encounter a CMake error).*

## 4. Stopping the Environment

When you are done working, you can safely shut down the simulation and free up system resources by running this from your Windows PowerShell in the project directory:
```powershell
docker compose down
```
