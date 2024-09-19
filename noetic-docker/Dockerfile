# Use arm64v8 ros image base
FROM arm64v8/ros:noetic

# Install rospy_tutorials
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rospy-tutorials \
    && rm -rf /var/lib/apt/lists/*
