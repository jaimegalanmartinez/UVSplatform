#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nlohmann/json.hpp>
// Library used: https://github.com/nlohmann/json
using json = nlohmann::json;
using std::placeholders::_1;

/** Fleet manager - ROS2 Node
  * @author: Jaime Galán Martínez
  */
class Fleet_manager : public rclcpp::Node
{
  public:
    Fleet_manager()
    : Node("fleet_manager")
    {
      //publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      subscription_VM_MissionPlan = this->create_subscription<std_msgs::msg::String>(
        "fleet_62a4b9fdaec3727326e3069a/missionPlan", 10, std::bind(&Fleet_manager::receive_mission_plan_callback, this,_1));

      subscription_UVS_webots_mission = this->create_subscription<std_msgs::msg::String>(
        "uv_webots/missionStatus", 10, std::bind(&Fleet_manager::receive_mission_status_callback, this,_1));
      
      publisher_UVS_mission_command = this->create_publisher<std_msgs::msg::String>("uv_webots/missionCommand", 10);

      publisher_VM_fleetStatus = this->create_publisher<std_msgs::msg::String>("fleetStatus_62a4b9fdaec3727326e3069a", 10);
     
    }

  private:
    rclcpp::TimerBase::SharedPtr one_off_timer, one_off_timer1;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_VM_fleetStatus;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_UVS_mission_command;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_UVS_status;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_VM_MissionPlan;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_UVS_webots_mission;
    //Vector storing the mission commands received
    std::vector<std::string> commands_array;
  
    void receive_mission_plan_callback(const std_msgs::msg::String & msg) 
    {
        auto mission_received_json = json::parse(msg.data.c_str());
        std::string vehicle_id = mission_received_json["vehicle_id"].dump();
        //auto fleet_id_mission = mission_received_json["fleet_id"].dump().c_str();
        std::string mission_plan_name = mission_received_json["mission_plan"]["name"].dump();
        std::string mission_commands = mission_received_json["mission_plan"]["mission_commands"].dump();
        //std::string command_1 = mission_received_json["mission_plan"]["mission_commands"].at(0)["name"].dump();
        short int number_commands = mission_received_json["mission_plan"]["mission_commands"].size();
        
        //Vector storing the mission commands received
        //Assure the array of commands is empty
        commands_array.clear();
        //Fill vector with the commands retrieved from the mission
        for (auto& command : mission_received_json["mission_plan"]["mission_commands"].items()){
            commands_array.emplace_back(command.value()["name"].dump());
            //std::cout << "key: " << command.key() << ", value:" << command.value()["name"] << '\n';
        }
    
        //MISSION INFORMATION RECEIVED
        RCLCPP_INFO(this->get_logger(), "Mission plan %s received for the vehicle: %s", mission_plan_name.c_str(), vehicle_id.c_str());
        RCLCPP_INFO(this->get_logger(), "with %d commands:", number_commands);
        //Print all commands stored in commands_array (std::vector)
        for (uint8_t i = 0; i < commands_array.size(); i++){
          //https://stackoverflow.com/questions/20326356/how-to-remove-all-the-occurrences-of-a-char-in-c-
          //Delete "" from all elements in commands_array
          commands_array[i].erase(std::remove(commands_array[i].begin(), commands_array[i].end(), '"'), commands_array[i].end());
           RCLCPP_INFO(this->get_logger(), "c%d: %s", i, commands_array[i].c_str());
        }
        
        auto message = std_msgs::msg::String();
        //message.data = std::string("Fleet manager: ") + commands_array[0].c_str();
        message.data = commands_array[0].c_str();
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
        publisher_UVS_mission_command->publish(message);
        
      
    }

    void receive_mission_status_callback(const std_msgs::msg::String & msg)
    {   
        auto messageToVehicle = std_msgs::msg::String();
        RCLCPP_INFO(this->get_logger(), "Received mission status from UVS Controller: %s", msg.data.c_str());

        if(msg.data == "NAV_TAKEOFF command executed successfully"){
          RCLCPP_INFO(this->get_logger(), "Sending NAV_WAYPOINT command");
          messageToVehicle.data = commands_array[1].c_str();
          publisher_UVS_mission_command->publish(messageToVehicle);
          //Send message to vehicle manager that the mission is running
          auto messageToVM = std_msgs::msg::String();
          messageToVM.data = std::string("webots mission is running");
          publisher_VM_fleetStatus->publish(messageToVM);

        }else if(msg.data == "NAV_WAYPOINT command executed successfully"){
          RCLCPP_INFO(this->get_logger(), "Sending NAV_MEASURE command");
          messageToVehicle.data = commands_array[2].c_str();
          publisher_UVS_mission_command->publish(messageToVehicle);

        }else if(msg.data == "NAV_MEASURE command executed successfully"){
          RCLCPP_INFO(this->get_logger(), "Sending NAV_HOME command");
          messageToVehicle.data = commands_array[3].c_str();
          publisher_UVS_mission_command->publish(messageToVehicle);

        }else if(msg.data == "NAV_HOME command executed successfully"){
          RCLCPP_INFO(this->get_logger(), "Sending NAV_LAND command");
          messageToVehicle.data = commands_array[4].c_str();
          publisher_UVS_mission_command->publish(messageToVehicle);

        }else if(msg.data == "webots mission finished"){
          auto messageToVM = std_msgs::msg::String();
          messageToVM.data = std::string("webots mission finished");
          publisher_VM_fleetStatus->publish(messageToVM);
        }
        

    }
    
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
  rclcpp::spin(std::make_shared<Fleet_manager>());
  rclcpp::shutdown();
  return 0;
}
