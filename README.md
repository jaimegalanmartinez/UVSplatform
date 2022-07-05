# UVS platform
Master's thesis: Integration of Data Distribution Service (DDS) enabled vehicles in IoT. Master's thesis developed for the MSc IoT UPM 2021/2022. Design of a system able to deliver
missions to unmanned vehicles, supervise their execution, and satisfy the system real-time 
requirements according to the generic architecture of IoT systems.
Furthermore, a small proof-of-concept has been developed to show the design feasibility by
using an open-source robot simulator as Webots.

### Author
Jaime Galán Martínez

### Prerequisites
You should have the following prerequisites installed in order to test the prototype:
- ROS2 in Ubuntu 20.04 (to run Webots drone), specifically this project was developed using ROS2 Galactic.
- Webots simulator. Version: R2022a
- Docker - Version used 20.10.12
- Node.js - Version used v16.15.0
- npm - Version used 8.5.5
- Flutter SDK - Version used: 3.0.1
- Dart - Version used: 2.17.1
- Android Studio - Version used: Chipmunk | 2021.2.1 Patch 1

Also you need to modify the file: uvsp_app/lib/services/http_service.dart and change the value of uriMissionManager
to the specific IP address of the PC where is running locally the Mission Manager and Vehicle Manager.
Example: static const String uriMissionManager = "http://IP:3000"; where IP has the following format: X.X.X.X
As default is established my mobile phone network with IP: 192.168.43.74

### Instructions
Here are the detailed instructions to test the prototype:

#### Authentication Server
For register, login and logout into the system, it is used Firebase Authentication.

#### Mission manager
Inside the backend/mission_manager folder, execute the following command:
```bash 
npm run start
```
#### Vehicle manager
Inside the backend/vehicle_manager folder do the following steps in a bash terminal:
 - Source ROS2 environment with the following command:
    ```bash 
    source /opt/ros/galactic/setup.bash
    ``` 
 - Execute the command:
    ```bash 
    npm run start
     ``` 
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
```bash
ros2 run uvs_fleet_manager fleet_manager
```
Run another interactive shell to access the container ros2_ubuntu for running the UVS_Controller
```bash
docker exec -it ros2_ubuntu bash
```
After that, run the UVS Controller ros2 node
```bash
. install/local_setup.bash
ros2 run uvs_pkg uvs_controller
```
#### Webots drone simulator
Run the following commands to source ROS2 env and to start the webots drone in another bash terminal:
```bash
source /opt/ros/galactic/setup.bash
ros2 launch webots_ros2_mavic robot_launch.py
```
#### Mobile application - UVS Platform
Tested using a mobile phone using Android 10. Start the UVS Platform app from your mobile.
1. Sign up in the system if you don't have already created your user.
2. Log in with your credentials (email and password)
3. Automatically, the application will retrieve the vehicles availables for send a mission.
4. Select the vehicle "UAV Webots" by clicking in the button Request a mission.
5. It will appear a new screen with the mission plans available for that vehicle.
6. Select the unique mission plan available
7. It will appear a snackbar in the UI, allowing the user to confirm the mission plan selected in order to send the mission to the Webots drone.