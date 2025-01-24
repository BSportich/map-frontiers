# Start from the ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /opt/catkin_ws/src

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    vim \
    build-essential \
    python3-rosdep \
    python3-catkin-tools \
    libtool \
    && rm -rf /var/lib/apt/lists/*

# Clone the ROS package from GitHub
RUN git clone --single-branch --branch test/avenue https://github.com/BSportich/map-frontiers.git \
    && wstool init . /opt/catkin_ws/src/map-frontiers/voxfield_https.rosinstall \
    && wstool update

# Go back to the workspace root
WORKDIR /opt/catkin_ws/

# Initialize and build the Catkin workspace
RUN catkin config  --extend /opt/ros/noetic \
    && catkin build voxblox_ros 

# # Hacky hack
RUN mkdir -p /opt/catkin_ws/devel/.private/voxblox_map/include/ \
    && mkdir -p /opt/catkin_ws/devel/.private/map_frontiers/include/ \
    && catkin build map_frontiers

# # Source the setup.bash so that the package is available in the environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /opt/catkin_ws/devel/setup.bash" >> ~/.bashrc

# # Set the entrypoint
CMD ["bash", "-c"]
