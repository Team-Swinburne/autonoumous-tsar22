#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "Path_Finding_No_Ros.cpp"
#include "Path_State.cpp"
#include "Directions.cpp"

using namespace std::chrono_literals;

// This class is the class that is used to interact with ROSS2, turtlebot3, and gazebo
class MinimalPublisher : public rclcpp::Node
{
  public:
    float turning = 0;
    PathPlanning Planning;
    vector<Direction> directions;
    MinimalPublisher()
    : Node("Path_Finding")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
      i = 0.0;
      Planning.PlanPath(); 
      timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::publish_message, this));
    }

  private:
    void publish_message()
    {
      cout << "Previous Heading: " << previousHeading << "Current Heading: " << currentHeading << endl;
      cout << "Point along Path" << pointalongpath << endl;
      cout << "turning: "<< turning << endl;
      if (pointalongpath < Planning.directions.size()) {
      if (pointalongpath > 0) {
        previousHeading = Planning.directions.at(pointalongpath -1)->VehicleHeading;
        currentHeading = Planning.directions.at(pointalongpath)->VehicleHeading;
        // Calculates the turning velocity in radians per second converted from the vehicles degrees that it needs to turn
        if (currentHeading != previousHeading) {
          int headingdiff = 0;
          int wraparounddiff = 0;
          if (previousHeading > currentHeading) {
            headingdiff = previousHeading - currentHeading;
            // Handles wrap around of 360 / 0 degrees
            if (headingdiff > 45) {
              wraparounddiff = 360 + currentHeading;
              headingdiff = wraparounddiff - previousHeading;
            }
            turning = headingdiff * (3.14159 / 180);
          } else {
            headingdiff = previousHeading - currentHeading;
            // Handles wrap around of 360 / 0 degrees
            if (headingdiff < -45) {
              wraparounddiff = 360 + previousHeading;
              headingdiff = currentHeading - wraparounddiff;
            }
            turning = headingdiff * (3.14159 / 180);
          }
        } else {
          turning = 0;
        }
      } 
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.1; 
        message.angular.z = turning;
        RCLCPP_INFO(this->get_logger(), "Sending - Linear Velocity : '%f', Angular Velocity : '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);
        i += 1; 
        pointalongpath++;
      } else {
       pointalongpath = 0;
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float i;
    int pointalongpath = 0;
    int previousHeading = 0, currentHeading = 0;
};

int main(int argc, char ** argv)
{
  
  // PathPlanning Planning;
  // vector <Direction> directions;

  // Planning.Start_State.PrintPosition();

  // Planning.PlanPath(); 
  rclcpp::init(argc, argv);
  while (true) {
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  }

  rclcpp::shutdown();

  return 0;
}
