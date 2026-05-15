FROM osrf/ros:humble-desktop

# Add universe repo and install all dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    fluxbox \
    wget \
    curl \
    git \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    python3-opencv \
    libgl1-mesa-dri \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

# Force software OpenGL rendering (no GPU needed)
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DISPLAY=:0

# Install Python ML dependencies (Skip ultralytics/torch for now to speed up build)
# RUN pip3 install --no-cache-dir ultralytics 
RUN pip3 install --no-cache-dir opencv-python-headless numpy

# Setup NoVNC
RUN ln -s /usr/share/novnc/vnc_lite.html /usr/share/novnc/index.html

# Setup workspace
WORKDIR /workspace

# Copy the start script
COPY start.sh /start.sh
RUN chmod +x /start.sh

# Expose NoVNC port
EXPOSE 8080

ENTRYPOINT ["/start.sh"]
