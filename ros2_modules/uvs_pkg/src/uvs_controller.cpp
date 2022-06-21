#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

//Twist  -> Velocity in 3-dimensional free space broken into its linear and angular parts.
//UVS (Unmanned vehicle system)
class UVS_controller : public rclcpp::Node
{
  public:
    UVS_controller()
    : Node("uvs_controller")
    {
      publisher_vehicle = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      publisher_mission_status = this->create_publisher<std_msgs::msg::String>("uv_webots/missionStatus", 10);
      subscription_Fleet_missionCommands= this->create_subscription<std_msgs::msg::String>(
        "uv_webots/missionCommand", 10, std::bind(&UVS_controller::receive_mission_commands_callback, this,_1));
      //10 -depth of publisher message queue
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vehicle;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_mission_status;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_Fleet_missionCommands;

    void wait_for_sec(int32_t seconds){
      rclcpp::Time now = this->get_clock()->now();
      rclcpp::Time aux = now + rclcpp::Duration(seconds,0);
      while (now < aux){ //Wait seconds duration
        now = this->get_clock()->now();
      }

    }

    void receive_mission_commands_callback (const std_msgs::msg::String & msg) {
      auto mission_status = std_msgs::msg::String();
      auto speed_vehicle = geometry_msgs::msg::Twist();
      const float ONE_SEC_NS = 1000000000.0;
      rclcpp::Time init_mission_time;
      rclcpp::Time end_mission_time;

      RCLCPP_INFO(this->get_logger(), "Received command from Fleet Manager: %s", msg.data.c_str());
      //RCLCPP_INFO(this->get_logger(), "Received command from Fleet Manager: '%s'", msg.data.c_str());

      if(msg.data == "NAV_TAKEOFF"){
        RCLCPP_INFO(this->get_logger(), "Starting webots mission with command: %s", msg.data.c_str());
        init_mission_time = this->get_clock()->now();
        mission_status.data = "NAV_TAKEOFF command executed successfully";
        publisher_mission_status->publish(mission_status);

      } else if (msg.data == "NAV_WAYPOINT"){
        command_nav_waypoint(speed_vehicle);
        mission_status.data = "NAV_WAYPOINT command executed successfully";
        publisher_mission_status->publish(mission_status);

      } else if (msg.data == "NAV_MEASURE"){
        command_nav_measure(speed_vehicle);
        mission_status.data = "NAV_MEASURE command executed successfully";
        publisher_mission_status->publish(mission_status);

      }else if (msg.data == "NAV_HOME"){
        command_nav_home(speed_vehicle);
        mission_status.data = "NAV_HOME command executed successfully";
        publisher_mission_status->publish(mission_status);

      }else if(msg.data == "NAV_LAND"){
        command_nav_land(speed_vehicle);
        end_mission_time = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Mission executed in: %f s", (end_mission_time.nanoseconds()-init_mission_time.nanoseconds())/ONE_SEC_NS);
      
        mission_status.data = "webots mission finished";
        publisher_mission_status->publish(mission_status);

      }else{
        RCLCPP_INFO(this->get_logger(), " Error, command not expected: %s", msg.data.c_str());
      }
    }
    
    void measure_mission(){
      auto mission_status = std_msgs::msg::String();
      const float ONE_SEC_NS = 1000000000.0;
      auto speed_vehicle = geometry_msgs::msg::Twist();
      rclcpp::Time init_mission_time = this->get_clock()->now();
      //commands to be executed
      //NAV_WAYPOINT
      command_nav_waypoint(speed_vehicle);
      mission_status.data = "NAV_WAYPOINT command executed successfully";
      publisher_mission_status->publish(mission_status);
      //NAV_MEASURE
      command_nav_measure(speed_vehicle);
      mission_status.data = "NAV_MEASURE command executed successfully";
      publisher_mission_status->publish(mission_status);
      //NAV_HOME
      command_nav_home(speed_vehicle);
      mission_status.data = "NAV_HOME command executed successfully";
      publisher_mission_status->publish(mission_status);
      //NAV_LAND
      command_nav_land(speed_vehicle);
      rclcpp::Time end_mission_time = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "Mission executed in: %f s", (end_mission_time.nanoseconds()-init_mission_time.nanoseconds())/ONE_SEC_NS);
      
      mission_status.data = "webots mission finished";
      publisher_mission_status->publish(mission_status);

    }

    void command_nav_waypoint(geometry_msgs::msg::Twist &message){
      // Move to a specific location
      //auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1.5;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Starting to execute command NAV_WAYPOINT");
      publisher_vehicle->publish(message);
      wait_for_sec(5);
      //Stop
      message.linear.x = 0.0;
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);
      //Increase height
      message.linear.z = 3.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.z);
      publisher_vehicle->publish(message);
      wait_for_sec(8);
      //Stop
      message.linear.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.z);
      publisher_vehicle->publish(message);
      wait_for_sec(1);
      //Turn to left
      message.angular.z = 0.5;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.angular.z);
      publisher_vehicle->publish(message);
      wait_for_sec(2);
      //Stop turn
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.angular.z);
      publisher_vehicle->publish(message);
      wait_for_sec(1);
      //Move forward
      message.linear.x = 2.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(20);
      //Stop
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);

    }

    void command_nav_measure(geometry_msgs::msg::Twist &message){
      RCLCPP_INFO(this->get_logger(), "Starting to execute command NAV_MEASURE");
      //Descend to measure
      message.linear.z = -2.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(2);
      //Descending less speed
      message.linear.z = -1.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);
      //Stop
      message.linear.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(4);
      //Take previous height
      //Up
      message.linear.z = 1.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);

      message.linear.z = 2.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(5);
      //Stop
      message.linear.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);
      //Turn left
      message.angular.z = 0.6;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(8);
      //Stop
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);
    }

    void command_nav_home(geometry_msgs::msg::Twist &message){
       RCLCPP_INFO(this->get_logger(), "Starting to execute command NAV_HOME");
       //Return to base
      message.linear.x = 3.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(12);
      message.linear.x = 1.38;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(3);
      //Stop
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);

      //Turn right
      message.angular.z = -0.368;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(3);

       //Stop
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);

      //Move forward
      message.linear.y = 0;
      message.linear.x = 1.88;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      //wait_for_sec(4);
      wait_for_sec(6);

       //Stop
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(2);
    }

    void command_nav_land(geometry_msgs::msg::Twist &message){
       RCLCPP_INFO(this->get_logger(), "Starting to execute command NAV_LAND");
      //Descending less speed
      message.linear.z = -1.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(4);

      //Move left
      message.linear.y = 0.16;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(2);

      //Stop
      message.linear.z = 0.0;
      message.linear.y = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(2);

      //Turn right to back to initial position
      message.angular.z = -0.75;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(6.8);

      //Turn right to back to initial position
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_vehicle->publish(message);
      wait_for_sec(1);

    }
      // Make a measure
      //  1. Descend to measure
      //  2. Measure
      //  3. Take previous height
      // Return to base
      // Land in base
    //Base position
    //x = 0.0616
    //y = 0.0211
    //z = 0  -init in 0.78408
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //Create a default single-threaded executor and spin the specified node.
  //make_shared --> Allocates and constructs an object of type T passing args to its constructor, 
  //and returns an object of type shared_ptr<T> that owns and stores a pointer to it (with a use count of 1).
  // https://www.cplusplus.com/reference/memory/make_shared/
  //auto node = std::make_shared<UVS_controller>();
  rclcpp::spin(std::make_shared<UVS_controller>());
  rclcpp::shutdown();
  return 0;
}
