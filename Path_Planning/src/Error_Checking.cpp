#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string.h>

// to run this build the package and enter "ros2 run SEPB_Project_34 Error_Checking"
using namespace std::placeholders;

//global variable to keep track of robot locations
float expected_x = 0;
float expected_z = 0;
float current_x = 0;
float current_z = 0;
float difference_x = 0;
float difference_z = 0;


class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();

    // create a subscription to turtlebot3 odometry
    mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
                    "odom", qos,
                    std::bind(&MinimalSubscriber::odomCallback, this, _1) );

    // create subscription to control node
    conSub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:

  // set the expected variable by adding velocity to current location
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {

    expected_x = current_x + msg->linear.x;
    expected_z = current_z + msg->angular.z;
    //RCLCPP_INFO(this->get_logger(), "Expected location: X: '%f' , Z : %f'", expected_x, expected_z);
  }

  // get current location then get the differences between location
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        // current locations
        current_x = msg->pose.pose.position.x;
        current_z = msg->pose.pose.position.z;

        difference_x = expected_x - current_x;
        difference_z = expected_z - current_z;

          //print out
          RCLCPP_INFO(get_logger(), "Current error is : X: %.2f Z: %.2f",
                   difference_x, difference_z);

    }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr conSub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
