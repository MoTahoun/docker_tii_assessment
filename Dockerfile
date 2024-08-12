# Use the official ROS base image
FROM ros:noetic-ros-base

# Install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-pip \
    ros-noetic-rviz \
    ros-noetic-rosbag \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-msgs \
    ros-noetic-perception \
    ros-noetic-perception-pcl \
    ros-noetic-rqt* \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Create a catkin workspace
WORKDIR /workspace

# Build the catkin workspace
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the setup.bash file
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash exec \"$@\"", "bash"]
RUN echo "source /workspace/tii_ws/devel/setup.bash" >> ~/.bashrc
CMD ["bash", "-c", "source /workspace/tii_ws/devel/setup.bash exec \"$@\"", "bash"]

# SHELL ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash"]

# Set the entrypoint to the launch file
# ENTRYPOINT ["roslaunch", "pointcloud_processor", "pointcloud_processor.launch"]
