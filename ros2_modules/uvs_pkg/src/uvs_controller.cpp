#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

//Twist  -> Velocity in 3-dimensional free space broken into its linear and angular parts.
//UVS (Unmanned vehicle system)
class UVS_controller : public rclcpp::Node
{
  public:
    UVS_controller()
    : Node("uvs_controller"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      //10 -depth of publisher message queue
      //timer_ = this->create_wall_timer(
      //500ms, std::bind(&UVS_controller::timer_callback, this));
      speed_ = 0.5;
      turn_ = 1.0;
      x_ = 0.0;
      y_ = 0.0;
      z_ = 0.0;
      th_ = 0.0;
     
      //land();
      //send_command();
      measure_mission();
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
    double speed_, turn_, x_, y_, z_, th_;
    

    void wait_for_sec(int32_t seconds){
      rclcpp::Time now = this->get_clock()->now();
      rclcpp::Time aux = now + rclcpp::Duration(seconds,0);
      while (now < aux){ //Wait seconds duration
        now = this->get_clock()->now();
      }

    }

    void measure_mission(){
      /*
      NAV_TAKEOFF - Automatically
      NAV_WAYPOINT
      //In place for measure
      NAV_LAND
      NAV_TAKEOFF
      NAV_HOME
      NAV_LAND
      */
      //One second expressed in nanoseconds
      const float ONE_SEC_NS = 1000000000.0;
      // Take off
      
      // Move to a specific location
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1.5;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.z = 0.0;
      rclcpp::Time init_mission_time = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(5);
      //Stop
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);
      //Increase height
      message.linear.z = 3.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(8);
      //Stop
      message.linear.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);
      //Turn to left
      message.angular.z = 0.5;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(2);
      //Stop turn
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);
      //Move forward
      message.linear.x = 2.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(20);
      //Stop
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);
      //Descend to measure
      message.linear.z = -2.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(2);
      //Descending less speed
      message.linear.z = -1.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);
      //Stop
      message.linear.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(4);
      //Take previous height
      //Up
      message.linear.z = 1.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);

      message.linear.z = 2.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(5);
      //Stop
      message.linear.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);
      //Turn left
      message.angular.z = 0.6;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(8);
      //Stop
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);

       //Return to base
      message.linear.x = 3.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(12);
      message.linear.x = 1.38;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(3);
      //Stop
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);

      //Turn right
      message.angular.z = -0.368;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(3);

       //Stop
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);

      //Move forward
      message.linear.y = 0;
      message.linear.x = 1.88;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(4);

       //Stop
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(2);

      //Descending less speed
      message.linear.z = -1.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(4);

      //Move left
      message.linear.y = 0.16;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(2);

      //Stop
      message.linear.z = 0.0;
      message.linear.y = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(2);

      //Turn right to back to initial position
      message.angular.z = -0.75;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(6.8);

      //Turn right to back to initial position
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      wait_for_sec(1);

      rclcpp::Time end_mission_time = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "Mission executed in: %f s", (end_mission_time.nanoseconds()-init_mission_time.nanoseconds())/ONE_SEC_NS);
      // Make a measure
      //  1. Descend to measure
      //  2. Measure
      //  3. Take previous height
      // Return to base
      // Land in base
    }
    void send_command(){
      
      auto message = geometry_msgs::msg::Twist();
        //message.data = "Hello, world! " + std::to_string(count_++);
        //message.linear.x  x, y, z
        //message.angular.z x, y, z
        message.linear.x = 1.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
        publisher_->publish(message);
        //nanoseconds 
        //const ONE_SEC = 1000000000;
        //sleep_for(ONE_SEC);
      one_off_timer = this-> create_wall_timer(5s, [this](){
        printf("in one_off_timer callback\n");
        auto message = geometry_msgs::msg::Twist();
        //message.data = "Hello, world! " + std::to_string(count_++);
        //message.linear.x  x, y, z
        //message.angular.z x, y, z
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
        publisher_->publish(message);
        this->one_off_timer->cancel();
        
      });
      
      //one_off_timer->cancel();
      this->one_off_timer->reset();

      this->one_off_timer = this-> create_wall_timer(8s, [this](){
        printf("in one_off_timer1 callback\n");
        auto message = geometry_msgs::msg::Twist();
        //message.data = "Hello, world! " + std::to_string(count_++);
        //message.linear.x  x, y, z
        //message.angular.z x, y, z
        message.linear.x = 1.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
        publisher_->publish(message);
        this->one_off_timer->cancel();

      });
      
    }
    //Base position
    //x = 0.0616
    //y = 0.0211
    //z = 0  -init in 0.78408
    void land(){
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1.0;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Time now: '%f'", this->now().seconds());
      
      wait_for_sec(5);

      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
    
    }

    void take_off(){

    }
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
