# Use the official ROS Noetic base image
FROM ros:noetic


# Install additional dependencies for your C++ application
RUN apt-get update && apt-get install -y \
    g++ \
    cmake \
    libcpprest-dev

# Create a directory for your C++ application
WORKDIR /app

# Copy your C++ source code into the container
COPY restTest.cpp /app/
COPY CMakeLists.txt /app/
COPY src/* /app/src/
COPY include/* /app/include/

# Set the shell to Bash and source ROS setup.bash before running CMake
SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir build && cd build && cmake .. && make

# Expose the port your application will listen on
EXPOSE 8080

# Start the ROS master, then your C++ application when the container starts
CMD ["bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && roscore & ./build/RosRestBridge"]
