FROM osrf/ros:jazzy-desktop-full

# Install dependencies
RUN apt-get update -y && \
    apt-get install -y curl python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace

# Copy necessary files
COPY . .

# Install ROS dependencies

RUN rosdep install --from-paths . --ignore-src -ry || true

# # Build the project
# RUN . /opt/ros/jazzy/setup.sh && colcon build

# Source the setup script
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Run bash shell
CMD ["/bin/bash"]