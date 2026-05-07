FROM ros:humble

# Set working directory
WORKDIR /workspace

# Install Python and required build tools
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 bag tools and Python dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-py \
    ros-humble-rosbag2-storage \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-cv-bridge \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for data processing
RUN pip3 install --upgrade pip && \
    pip3 install \
    'numpy<2' \
    pandas \
    tqdm \
    pyyaml

# Copy repository contents (bag files are excluded via .dockerignore)
COPY . /workspace/

# Make scripts executable
RUN chmod +x /workspace/print_topics.py 2>/dev/null || true

# Set ROS environment variables
ENV ROS_DISTRO=humble

# Create entrypoint script that sources ROS2 and creates user entry
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'export HOME=/tmp' >> /entrypoint.sh && \
    echo 'USER_ID=$(id -u)' >> /entrypoint.sh && \
    echo 'GROUP_ID=$(id -g)' >> /entrypoint.sh && \
    echo 'USERNAME=${USER:-appuser}' >> /entrypoint.sh && \
    echo 'echo "$USERNAME:x:$USER_ID:$GROUP_ID::/tmp:/bin/bash" >> /etc/passwd' >> /entrypoint.sh && \
    echo 'echo "appgroup:x:$GROUP_ID:" >> /etc/group' >> /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Make /etc/passwd and /etc/group writable for non-root users
RUN chmod 666 /etc/passwd /etc/group

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
