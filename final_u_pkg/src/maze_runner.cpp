#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>

using std::placeholders::_1;


class MazeRunnerNode : public rclcpp::Node {
public:
    MazeRunnerNode() : Node("maze_runner_node") {
        // QoS 설정
        rclcpp::QoS qos_profile_sensor = rclcpp::QoS(rclcpp::SensorDataQoS());
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile_sensor, 
            std::bind(&MazeRunnerNode::scan_callback, this, std::placeholders::_1)
        );
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MazeRunnerNode::odom_callback, this, _1)
        );
            
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
    }

private:
    
    enum class RobotState {
        EXPLORING,
        ESCAPING
    };

    RobotState current_state_ = RobotState::EXPLORING;

    double current_yaw_ = 0.0;
    double target_escape_yaw_ = 0.0;

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (current_state_ == RobotState::EXPLORING) {
            run_exploring_mode(scan);
        }
        else if (current_state_ == RobotState::ESCAPING) {
            run_escaping_mode();
        }
    }

    void run_exploring_mode(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        geometry_msgs::msg::Twist cmd;
        std::vector<float> ranges = scan->ranges;
        
        // 1. 정면 안전 거리 확인 (약 정면 +-30도)
        float min_front_dist = 999.0;
        float max_dist = 0.0;
        int best_idx = -1;
        float max_cost = -999.0;

        for (size_t i = 0; i < ranges.size(); ++i) {
            float angle = scan->angle_min + (i * scan->angle_increment);
            float r = ranges[i];
            if (std::isinf(r) || std::isnan(r)) r = scan->range_max;

            // 정면 충돌 감지
            if (std::abs(angle) < 0.5 && r > 0.0) { 
                if (r < min_front_dist) min_front_dist = r;
            }

            // cost func 적용
            if (std::abs(angle) < 1.48) { 
                if (r > max_dist) max_dist = r; // 막힌 길 판단

                // 거리가 멀수록 +, 직진 방향일수록 +
                float cost = r - (1.2 * std::abs(angle));
                if (cost > max_cost) {
                    max_cost = cost;
                    best_idx = i;
                }
            }
        }

        // 막힌 길 감지
        if (min_front_dist < 0.3 && max_dist < 0.45) {
            RCLCPP_WARN(this->get_logger(), "막힌길");
            target_escape_yaw_ = normalize_angle(current_yaw_ + M_PI);
            current_state_ = RobotState::ESCAPING;
            return; 
        }

        // 충돌 방지 로직
        if (min_front_dist < 0.25) {
            cmd.linear.x = 0.0;
            
            float left_sum = 0.0, right_sum = 0.0;
            for (size_t i = 0; i < ranges.size(); ++i) {
                float angle = scan->angle_min + (i * scan->angle_increment);
                float r = (std::isinf(ranges[i]) || std::isnan(ranges[i])) ? scan->range_max : ranges[i];
                if (angle > 0 && angle < 1.5) left_sum += r;   
                if (angle < 0 && angle > -1.5) right_sum += r; 
            }
            
            cmd.angular.z = (left_sum > right_sum) ? 0.6 : -0.6;
            cmd_pub_->publish(cmd);
            return; 
        }

        // 일반 주행 제어
        if (best_idx != -1) {
            float target_angle = scan->angle_min + (best_idx * scan->angle_increment);
            cmd.angular.z = 0.8 * target_angle; 

            if (std::abs(target_angle) < 0.3) { 
                cmd.linear.x = 0.12; 
            } else if (std::abs(target_angle) < 0.6) { 
                cmd.linear.x = 0.05; 
            } else {
                cmd.linear.x = 0.0;  
            }
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
        }

        cmd_pub_->publish(cmd);
    }

    void run_escaping_mode() {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        
        double angle_diff = std::abs(normalize_angle(target_escape_yaw_ - current_yaw_));

        if (angle_diff < 0.15){
            RCLCPP_INFO(this->get_logger(), "180도 회전");
            current_state_ = RobotState::EXPLORING;
        } else {
            cmd.angular.z = 0.2;
        }
        cmd_pub_->publish(cmd);
        
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeRunnerNode>());
    rclcpp::shutdown();
    return 0;
}