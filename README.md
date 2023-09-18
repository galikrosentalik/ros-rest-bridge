# Using the Ros-Rest-bridge
**Building Docker image**
* docker build -t ros-rest-bridge .

**Running Docker image**
* docker run --network="host" -e "LOG_TO_CONSOLE=true" ros-rest-bridge

# Testing the bridge
**In order to attach to the running docker**
* docker exec -it [docker_id] /bin/bash (may get the docker_id by running docker ps command)

**Inside the docker container may run**
* python3 example/Subscriber.py topic_a topic_b
  * may add topics to subscribe
* python3 example/Publisher.py topic_a topic_b
  * may add topics to subscribe
  * will randomly generate msgs to topics
 
**The package can be checked manually using the browsers GET & POST commands**

    
