# Use an official ROS 2 Humble base image
FROM ros:humble

# Install necessary packages and tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# Use Bash as the default shell for subsequent RUN commands.
SHELL ["/bin/bash", "-c"]

# Initialize rosdep (remove the default file first to avoid errors)
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

# Set working directory
WORKDIR /autonomous_ROS

# Copy the project files into the container
COPY . /autonomous_ROS

# Build the workspace with the RViz plugin disabled
RUN . /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DBUILD_RVIZ_PLUGIN=OFF

# Set the container's entrypoint to launch your unified launch file
CMD ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /autonomous_ROS/install/setup.bash && ros2 launch my_robot_slam all_nodes_launch.py"]
