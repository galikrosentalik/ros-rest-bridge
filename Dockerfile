# Use an official Ubuntu runtime as a parent image
FROM ubuntu:20.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update package list and install necessary dependencies
RUN apt-get update -y && \
    apt-get install -y \
    g++ \
    cmake \
    libcpprest-dev

# Create a directory for your C++ application
WORKDIR /app

# Copy your C++ source code into the container
COPY restTest.cpp /app/
COPY CMakeLists.txt /app/

RUN echo "1"
# Build your C++ application
RUN mkdir build && cd build && cmake .. && make

# Expose the port your application will listen on
EXPOSE 8080

# Start your C++ application when the container starts
CMD ["./build/main"]

