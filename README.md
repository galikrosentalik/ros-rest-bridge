# Using the Ros-Rest-bridge
**Building Docker image**
* ```docker build -t ros-rest-bridge .```

**Running Docker image**
* ```docker run --network="host" -e "LOG_TO_CONSOLE=true" ros-rest-bridge```

# Testing the bridge
**In order to test with automated python scripts**
* In order to attach to the running docker
  * ```docker exec -it [docker_id] /bin/bash``` (may get the docker_id by running docker ps command)

* Inside the docker container may run
  * ```python3 example/Subscriber.py topic_a topic_b```
    * Topics to subscribe need to be added to the command
  * ```python3 example/Publisher.py topic_a topic_b```
    * Topics to Publish need to be added to the command
    * Will randomly generate msgs to topics
  * ```source /opt/ros/$ROS_DISTRO/setup.bash && python3 example/Publish_to_ros.py```
    * Will create a ROS publisher, will publish directly to ROS, not through the bridge
    * Will publish in 5 FPS random msg to first_topic




**The package can be tested manually using the browsers GET & POST commands**

    
