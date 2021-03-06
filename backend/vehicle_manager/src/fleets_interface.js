/**
 * Vehicle Manager - ROS2 Fleet interface 
 * @author: Jaime Galán Martínez
 * It was used the following library:
 * - https://github.com/RobotWebTools/rclnodejs 
 */

const rclnodejs = require('rclnodejs');


let publisher = '';
let node = '';
const fleet_id = '62a4b9fdaec3727326e3069a'; //Fleet Webots

(async () => {
    rclnodejs.init().then(() => {
        node = new rclnodejs.Node('VehicleManager');
        //publisher = node.createPublisher('std_msgs/msg/String', 'topicFleet');
        //publisher.publish(`Vehicle Manager ROS2 interface enabled`);
        const subscriberFleetStatus = node.createSubscription('std_msgs/msg/String', `fleetStatus_${fleet_id}`, (msg) => {
            console.log(`VEHICLE_MANAGER: Received status from FleetManager: ${typeof msg}`, msg);
            if (msg === 'webots mission finished'){
                console.log(`VEHICLE_MANAGER: Mission status received from FleetManager: ${typeof msg}`, msg);

            }else if(msg === 'webots mission is running'){
                console.log(`VEHICLE_MANAGER: Mission status received from FleetManager: ${typeof msg}`, msg);
            }
        });
        const subscriberFleetInfo = node.createSubscription('std_msgs/msg/String', `fleetInfo_${fleet_id}`, (msg) => {
            console.log(`VEHICLE_MANAGER: Received info from FleetManager: ${typeof msg}`, msg);
        });
        node.spin();
      });
})();



export function infoMessage(info){
    console.log(info);
};

export function publish(message, topic){
    publisher = node.createPublisher('std_msgs/msg/String', topic);
    publisher.publish(message);
}