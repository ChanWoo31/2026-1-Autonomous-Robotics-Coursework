
/*package dependencies -->CMakelist.txt, package.xml*/
#include <functional>
#include <memory>
#include <algorithm> // std::min, std::max
#include <chrono>
#include <thread>

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

#include <vector>
#include <numeric>
#include <limits>

using std::placeholders::_1;

class motor_control : public rclcpp::Node
{
    public:

    float set_vel;
    double roll_deg, pitch_deg;
    bool is_fallen;

    motor_control(): Node("motor_control")
    {
        auto default_qos = rclcpp::SensorDataQoS();
        
        set_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "set_velocity", 10,
            std::bind(&motor_control::set_velocity_callback,this,_1)
        );

        imu_angle_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", default_qos,
            std::bind(&motor_control::imu_callback,this,_1)
        );

        motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );

        lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&motor_control::lidar_callback,this,_1)
        );
    }
    
    void stop_motors()
    {
        auto stop_msg = geometry_msgs::msg::Twist();
        motor_pub_->publish(stop_msg);
    }

    private:

    int get_index_from_angle(int angle_deg, int ranges_size) const 
    {
        int normalized_angle = (angle_deg % 360 + 360) % 360;
        return (normalized_angle * ranges_size) / 360;
    }

    float get_safe_distance(const sensor_msgs::msg::LaserScan::SharedPtr& msg, int angle_deg) const 
    {
        int max_idx = msg->ranges.size();
        if (max_idx == 0) return 100.0f;
        
        int idx = get_index_from_angle(angle_deg, max_idx);
        float dist = msg->ranges[idx];
        
        if (!std::isnormal(dist) || dist < msg->range_min || dist > msg->range_max) {
            return 10.0f; 
        }
        return dist;
    }

    void set_velocity_callback(std_msgs::msg::Float64::SharedPtr _msg)
    {
        set_vel = _msg->data;
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr _msg)
    {
        tf2::Quaternion q(
            _msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        roll_deg = roll * 180 / M_PI;
        pitch_deg = pitch * 180 / M_PI;

        if (roll_deg > 45 || roll_deg < -45 || pitch_deg > 45 || pitch_deg < -45)
        {
            is_fallen = true;
            stop_motors();
        }
        else{
            is_fallen = false;
        }

        // RCLCPP_INFO(this->get_logger(), "%f", roll_deg);

    }

    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        if (is_fallen)
        {
            stop_motors();
            return;
        }

        int max_index = _msg->ranges.size();
        if (max_index == 0) return;

        float dist_front = get_safe_distance(_msg, 0);
        float dist_left_45 = get_safe_distance(_msg, 45);
        float dist_right_45 = get_safe_distance(_msg, 315);

        auto twist = geometry_msgs::msg::Twist();

        float danger_threshold = 0.30f; //안전거리

        // 비례 제어
        if (dist_front < danger_threshold) {
            // 정면이 막혔을 때 제자리 회전
            twist.linear.x = 0.0;
            twist.angular.z = (dist_left_45 > dist_right_45) ? 1.5 : -1.5;
        } 
        else {
            // 정면이 열려있을 때 주행하되, 좌우 벽과의 거리에 따라 조향각을 부드럽게 조정
            twist.linear.x = set_vel;
            
            // 에러 값: 왼쪽 벽과 오른쪽 벽의 거리 차이
            // 이 차이가 0이 되도록(미로 중앙을 타도록) 각속도 제어
            float error = dist_left_45 - dist_right_45;
            
            // 벽이 양쪽 다 감지되는 미로 내부일 경우에만 중앙 정렬 활성화 (P Gain: 1.0)
            if (dist_left_45 < 1.0 && dist_right_45 < 1.0) {
                twist.angular.z = 1.0 * error; 
            } else {
                twist.angular.z = 0.0;
            }
        }

        motor_pub_->publish(twist);
        
    }
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_velocity_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_angle_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control>();
    rclcpp::on_shutdown([node]() {
        node->stop_motors();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



