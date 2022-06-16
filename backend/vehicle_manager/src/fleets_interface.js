//https://github.com/RobotWebTools/rclnodejs

//import * as rclnodejs from 'rclnodejs';
const rclnodejs = require('rclnodejs');


let publisher = '';
(async () => {
    rclnodejs.init().then(() => {
        const node = new rclnodejs.Node('VehicleManager');
        publisher = node.createPublisher('std_msgs/msg/String', 'topicFleet');
        publisher.publish(`Hello ROS 2 from rclnodejs`);
        node.spin();
      });
})();



export function hola(info){
    console.log(info);
};

export function publish(message){
    //const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
    publisher.publish(message);
}