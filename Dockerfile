# Use the official ROS Noetic base image
FROM ros:noetic

# Install additional dependencies for your C++ application
RUN apt-get update && apt-get install -y \
    g++ \
    cmake \
    curl \
    nano \
    libcpprest-dev

# Create a directory for your C++ application
WORKDIR /app

# Copy your C++ source code into the container
COPY CMakeLists.txt /app/
COPY src/* /app/src/
COPY include/* /app/include/
COPY example/* /app/example/

SHELL ["/bin/bash", "-c"]

# Create a catkin workspace, initialize and configure the ROS package
RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    source /opt/ros/$ROS_DISTRO/setup.bash && catkin_init_workspace && \
    catkin_create_pkg ros_rest_interface roscpp std_msgs

# Copy the contents of /app to the ROS package directory
RUN cp -r /app/* ~/catkin_ws/src/ros_rest_interface/

# Build the ROS package
RUN cd ~/catkin_ws/ && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make

# Expose the port your application will listen on
EXPOSE 8080

CMD ["/bin/bash", "-ilc", "source ~/catkin_ws/devel/setup.bash && roscore & \
      sleep 1 && source ~/catkin_ws/devel/setup.bash && rosrun ros_rest_interface ros_rest_interface"]