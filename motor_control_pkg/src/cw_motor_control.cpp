#include <stdio.h>
/*package dependencies -->CMakelist.txt, package.xml*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;


class cw_motor_control : public rclcpp::Node
{
    public:
    float set_velocity_, set_orientation_;
    cw_motor_control()
    : Node("cw_motor_control")
    {
        auto default_qos = rclcpp::SensorDataQoS();
        lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&cw_motor_control::lidar_callback,this,_1)
        );
        motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    }
    void stop_motors()
    {
        auto stop_msg = geometry_msgs::msg::Twist(); // 모든 값 0.0으로 초기화됨
        motor_pub_->publish(stop_msg);
    }

    private:
    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        // geometry_msgs::msg::Twist twist;
        auto twist = geometry_msgs::msg::Twist();
        // int total_points = _msg->ranges.size();
        float distance_0 = _msg->ranges[0];
        float distance_45_right = _msg->ranges[685];
        float distance_45_left = _msg->ranges[45];
        // float velocity_, orientation_;
        if (distance_0 < 0.3){
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0;
            motor_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "전방 거리: '%f'", distance_0);
        }
        else if (distance_45_left < 0.3){
            twist.linear.x = 1.0;
            twist.angular.z = 2.0;
            motor_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "왼쪽 45도 거리: '%f'", distance_45_left);
        }
        else if (distance_45_right < 0.3){
            twist.linear.x = 1.0;
            twist.angular.z = -2.0;
            motor_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "오른쪽 45도 거리: '%f'", distance_45_right);
        }
        else{
            twist.linear.x = set_velocity_; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = set_orientation_;
        }
        motor_pub_->publish(twist);
        // RCLCPP_INFO(this->get_logger(), "전체개수: '%d'", total_points);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
};

int main(int argc, char * argv[])
{
    float velocity_, orientation_;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cw_motor_control>();
    
    while(rclcpp::ok()){
        scanf("%f %f", &velocity_, &orientation_);
        node->set_velocity_ = velocity_;
        node->set_orientation_ = orientation_;
        rclcpp::spin(node);
    }
    // rclcpp::spin(node);
    node->stop_motors();
    rclcpp::shutdown();
    return 0;
}
