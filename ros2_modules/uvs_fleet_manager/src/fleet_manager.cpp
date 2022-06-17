#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nlohmann/json.hpp>
//https://github.com/nlohmann/json
using json = nlohmann::json;
using std::placeholders::_1;

class Fleet_manager : public rclcpp::Node
{
  public:
    Fleet_manager()
    : Node("fleet_manager")
    {
      //publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      subscription_VM_MissionPlan = this->create_subscription<std_msgs::msg::String>(
        "fleet_62a4b9fdaec3727326e3069a/missionPlan", 10, std::bind(&Fleet_manager::receive_mission_plan_callback, this,_1));
     
    }

  private:
    rclcpp::TimerBase::SharedPtr one_off_timer, one_off_timer1;
    /*void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }*/
    //rclcpp::TimerBase::SharedPtr timer_;

    void receive_mission_plan_callback(const std_msgs::msg::String & msg) const
    {
        auto mission_received_json = json::parse(msg.data.c_str());
        auto vehicle_id = mission_received_json["vehicle_id"].dump().c_str();
        //auto fleet_id_mission = mission_received_json["fleet_id"].dump().c_str();
        auto mission_plan_name = mission_received_json["mission_plan"]["name"].dump().c_str();
        auto mission_commands = mission_received_json["mission_plan"]["mission_commands"].dump().c_str();
       
        //RCLCPP_INFO(this->get_logger(), "Received from Vehicle manager: '%s'", msg.data.c_str());
        RCLCPP_INFO(this->get_logger(), "Mission plan %s received for the vehicle: %s", mission_plan_name, vehicle_id);
        RCLCPP_INFO(this->get_logger(), "Commands: %s", mission_commands);


        
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_VM_fleetStatus;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_UVS_mission_command;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_UVS_status;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_VM_MissionPlan;


  
    
    void wait_for_sec(int32_t seconds){
      rclcpp::Time now = this->get_clock()->now();
      rclcpp::Time aux = now + rclcpp::Duration(seconds,0);
      while (now < aux){ //Wait seconds duration
        now = this->get_clock()->now();
      }

    }

};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //Create a default single-threaded executor and spin the specified node.
  //make_shared --> Allocates and constructs an object of type T passing args to its constructor, 
  //and returns an object of type shared_ptr<T> that owns and stores a pointer to it (with a use count of 1).
  // https://www.cplusplus.com/reference/memory/make_shared/
  //auto node = std::make_shared<UVS_controller>();
  rclcpp::spin(std::make_shared<Fleet_manager>());
  rclcpp::shutdown();
  return 0;
}
