
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

    float prev_error_ = 0.0f;
    rclcpp::Time last_time_;

    const float Kp_ = 1.0f;
    const float Kd_ = 0.1f;

    float get_sector_distance(const sensor_msgs::msg::LaserScan::SharedPtr& msg, int start_angle, int end_angle, bool use_min = false) const 
    {
        int max_idx = msg->ranges.size();
        if (max_idx == 0) return 10.0f;

        float sum = 0;
        int count = 0;
        float min_dist = std::numeric_limits<float>::max();

        for (int angle = start_angle; angle <= end_angle; ++angle) {
            int idx = get_index_from_angle(angle, max_idx);
            float dist = msg->ranges[idx];
            
            if (std::isnormal(dist) && dist >= msg->range_min && dist <= msg->range_max) {
                sum += dist;
                count++;
                if (dist < min_dist) min_dist = dist;
            }
        }

        if (count == 0) return 10.0f; // 범위 내 유효한 값이 없으면 열린 공간으로 간주
        return use_min ? min_dist : (sum / count);
    }

    int get_index_from_angle(int angle_deg, int ranges_size) const 
    {
        int normalized_angle = (angle_deg % 360 + 360) % 360;
        return (normalized_angle * ranges_size) / 360;
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

        auto current_time = this->now();
        if (last_time_.nanoseconds() == 0) {
            last_time_ = current_time;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        float dist_front = std::min(get_sector_distance(_msg, 0, 15, true), get_sector_distance(_msg, 345, 359, true));

        float dist_left = get_sector_distance(_msg, 30, 60, false);
        float dist_right = get_sector_distance(_msg, 300, 330, false);

        auto twist = geometry_msgs::msg::Twist();
        float danger_dist = 0.35f;

        if (dist_front < danger_dist) {
            twist.linear.x = 0.0;
            float front_left = get_sector_distance(_msg, 15, 45, false);
            float front_right = get_sector_distance(_msg, 315, 345, false);
            twist.angular.z = (front_left > front_right) ? 1.5 : -1.5;

            prev_error_=0.0f;
        }
        else if (dist_left > 1 && dist_right > 1){
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        else {
            twist.linear.x = set_vel;
            float error = dist_left - dist_right;

            float derivative = (error - prev_error_) / dt;

            if (dist_left < 1.5 && dist_right < 1.5) {
                twist.angular.z = (Kp_ * error) + (Kd_ * derivative);
            }
            else {
                twist.angular.z = 0.0;
            }
            prev_error_=error;
        }

        motor_pub_->publish(twist);
        last_time_ = current_time;
        
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



