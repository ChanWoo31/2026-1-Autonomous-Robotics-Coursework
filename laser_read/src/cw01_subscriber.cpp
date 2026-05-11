#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class cw01_subscriber : public rclcpp::Node
{
    public:
    cw01_subscriber()
    : Node("cw01_subscriber")
    {
        angle_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic", 10, std::bind(&cw01_subscriber::angle_callback, this, _1)
        );
        auto default_qos = rclcpp::SensorDataQoS();
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&cw01_subscriber::lidar_callback, this, _1)
        );
        target_angles_ = {0, 0, 0};
        last_states_ = {false, false, false}; //false 40 이상, true: 40cm 미만
    }
    private:
    void angle_callback(std_msgs::msg::Int32MultiArray::SharedPtr _msg) 
    {
        target_angles_[0] = _msg->data[0];
        target_angles_[1] = _msg->data[1];
        target_angles_[2] = _msg->data[2];
        // RCLCPP_INFO(this->get_logger(), "angle1: %d, angle2: %d, angle3: %d", target_angles_[0], target_angles_[1], target_angles_[2]);
    }
    
    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg) 
    {
        for (int i = 0; i<3; i++){
            int angle = target_angles_[i];

            float distance = _msg->ranges[angle];
            bool current_state = (distance < 0.4);
            if (current_state != last_states_[i]){
                std::string status;
                if (current_state == true){
                    status = "장애물 40cm 미만";
                } else {
                    status = "장애물 40cm 이상";
                }
                if (distance < 1.5){
                RCLCPP_INFO(this->get_logger(), "각도 변화: %d, 상태: %s, 거리: %f ", target_angles_[0], status.c_str(), distance);}
                last_states_[i] = current_state;

            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    std::vector<int> target_angles_;
    std::vector<bool> last_states_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<cw01_subscriber>();
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}