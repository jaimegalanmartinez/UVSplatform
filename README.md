# UVS platform
UVS Platform. Master's thesis developed for the MSc IoT UPM 2021/2022. Design of a system able to deliver
missions to unmanned vehicles, supervise their execution, and satisfy the system real-time 
requirements according to the generic architecture of IoT systems.
Furthermore, a small proof-of-concept has been developed to show the design feasibility by
using an open-source robot simulator as Webots

### Author
Jaime Galán Martínez

### Instructions

#### Mission manager
Inside the backend/mission_manager folder, execute the following command: npm run start

#### Vehicle manager
Inside the backend/vehicle_manager folder do the following steps in a bash terminal:
 - Source ROS2 environment with the following command: source /opt/ros/galactic/setup.bash
 - Execute the command: npm run start

#### ROS2 modules
To run the ROS2 modules: UVS Fleet Manager and UVS Controller you need to do the following steps:

Build the docker container if it has not been built already
```bash
docker build -f Dockerfile -t ros2_ubuntu:1.0 .
```
Run the container inside your host network (--net=host)
```bash
docker run --net=host --name ros2_ubuntu -it ros2_ubuntu:1.0
```

Inside the container execute the following command to source the ros2 workspace
```bash
. install/local_setup.bash
```
After that, run the Fleet manager ros2 node
ros2 run uvs_fleet_manager fleet_manager

Run another interactive shell to access the container ros2_ubuntu for running the UVS_Controller
```bash
docker exec -it ros2_ubuntu bash
```
After that, run the UVS Controller ros2 node
ros2 run uvs_pkg uvs_controller