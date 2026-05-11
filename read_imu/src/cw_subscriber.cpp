#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

using std::placeholders::_1;

class cw_subscriber : public rclcpp::Node
{
    public:
    double roll_deg;
    cw_subscriber()
    : Node("cw_subscriber")
    {
        angle_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(), std::bind(&cw_subscriber::imu_callback, this, _1)
        );
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&cw_subscriber::lidar_callback, this, _1)
        );
    }

    private:
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        roll_deg = roll * 180 / M_PI;
        // RCLCPP_INFO(this->get_logger(), "recieve msg = '%f'",roll_deg);
    }

    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        if (roll_deg >30){
            float distance1 = _msg->ranges[0];
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "30도 이상 = '%f' ", distance1);
        }
        if (roll_deg < -30){
            float distance2 = _msg->ranges[180];
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "30도 미만 = '%f' ", distance2);
        }
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cw_subscriber>());
    rclcpp::shutdown();
    return 0;
}