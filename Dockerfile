
# Use an official ROS 2 Humble base image
FROM ros:humble

# Install necessary packages and tools (for building your workspace)
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up rosdep
RUN rosdep init && rosdep update

# Set the working directory for your ROS 2 workspace
WORKDIR /autonomous_ROS

# Copy your project files into the container
COPY . /autonomous_ROS

# Build the workspace (you can also disable GUI plugins as needed)
RUN . /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DBUILD_RVIZ_PLUGIN=OFF

# Source the workspace setup file and set the entrypoint
# This launches your unified launch file on container startup.
CMD ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /autonomous_ROS/install/setup.bash && ros2 launch my_robot_slam all_nodes_launch.py"]
