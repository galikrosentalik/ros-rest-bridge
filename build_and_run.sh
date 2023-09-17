docker build -t ros-rest-bridge .
docker run --network="host" -e "LOG_TO_CONSOLE=true" ros-rest-bridge
