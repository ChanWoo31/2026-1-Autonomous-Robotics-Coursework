#include <stdio.h>
/*package dependencies -->CMakelist.txt, package.xml*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
int main(int argc, char * argv[])
{
 float velocity_, orientation_;
 rclcpp::init(argc,argv);
 auto node = rclcpp::Node::make_shared("motor_robot");
 // define publisher
 auto motor_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
 geometry_msgs::msg::Twist twist;
 while(rclcpp::ok()){
 // get the pressed key
 scanf("%f %f", &velocity_, &orientation_);
 twist.linear.x = velocity_; twist.linear.y = 0.0; twist.linear.z = 0.0;
 twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = orientation_;
 motor_->publish(twist);
 rclcpp::spin_some(node);
 }
 rclcpp::shutdown();
 return 0;
}