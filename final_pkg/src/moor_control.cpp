
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

        float distance_45_left_back = _msg->ranges[270];
        float distance_45_right_back = _msg->ranges[450];
        float distance_45_left = _msg->ranges[90];
        float distance_45_right = _msg->ranges[630];
        float distance_0_front = _msg->ranges[0];
        

        auto twist = geometry_msgs::msg::Twist();

        // 개수 확인.
        int max_index = _msg->ranges.size();
        if (max_index == 0) return;

        int start_deg = std::min(distance_45_left, distance_45_right);
        int end_deg = std::max(distance_45_left, distance_45_right);

        int start_idx = start_deg * 2;
        int end_idx = end_deg * 2;

        bool obstacle_detected = false;

        int min_idx = -1;
        float min_dist = 100.0f;

        for (int i = start_idx; i <= end_idx; ++i)
        {
            int real_idx = (i % max_index + max_index) % max_index;

            float distance = _msg->ranges[real_idx];

            if (std::isnormal(distance) && distance >= _msg->range_min && distance <= _msg->range_max)
            {
                if (distance < min_dist) {
                    min_dist = distance;
                    min_idx = real_idx; // 화면 출력을 위해 진짜 인덱스 기억
                }

                if (distance < 0.2){
                    obstacle_detected = true;
                }
            }
        }

        if (min_idx != -1){
            float min_angle = min_idx / 2.0f;

            if (min_angle > 180.0f) {
                min_angle -= 360.0f;
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "각도 %.1f도에서 최소거리 %.1fm", min_angle, min_dist
            );
        }

        
        

        // ii. 창2에서 넣은 두 개 값 사이 각도에 거리 최소가 20cm 이내인 경우 (-100~80도(입력된 라이더 센서 각도 두개) 안에 20cm 있는 경우 모터 정지, 그 외 부분의 경우 바퀴 별 모터 속도 변경 (장애물을 피하는 방향으로)
        if (obstacle_detected){
            stop_motors();
        }
        else if (distance_45_left < 0.2){
            twist.linear.x = set_vel;
            twist.angular.z = 0.5;
            motor_pub_->publish(twist);
        }
        else if (distance_45_right < 0.2){
            twist.linear.x = set_vel;
            twist.angular.z = -0.5;
            motor_pub_->publish(twist);
        }
        else {
            twist.linear.x = set_vel;
            twist.angular.z = 0.0;
            motor_pub_->publish(twist);
        }
        
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



