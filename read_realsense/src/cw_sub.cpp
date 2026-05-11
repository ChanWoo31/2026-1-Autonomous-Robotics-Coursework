#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class cw_sub : public rclcpp::Node
{
    public:
    cw_sub():Node("rgbd__")
    {
        num_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic", 10, std::bind(&cw_sub::num_callback, this, _1)
        );

        auto default_qos = rclcpp::SensorDataQoS();

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", default_qos,
            std::bind(&cw_sub::rgb_callback, this, _1)
        );

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", default_qos,
            std::bind(&cw_sub::depth_callback, this, _1)
        );

        gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/camera/gyro/sample", default_qos,
            std::bind(&cw_sub::gyro_callback, this, _1)
        );

        target_nums_ = {0, 0, 0, 0};
    }

    private:
    void rgb_callback(sensor_msgs::msg::Image::SharedPtr _msg)
    {
        // RCLCPP_INFO(this->get_logger(), "[0th] = '%d' [10000th] = '%d'", _msg->data[10001], _msg->data[10000]);
        int x = target_nums_[0] * target_nums_[1];
        int y = target_nums_[2] * target_nums_[3];

        size_t index = (y * _msg->step) + (x * 3);

        uint8_t r = _msg->data[index];
        uint8_t g = _msg->data[index + 1];
        uint8_t b = _msg->data[index + 2];
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "x, y: %d, %d => r: %d, g: %d, b: %d", x, y, r, g, b);

    }

    void depth_callback(sensor_msgs::msg::Image::SharedPtr _msg)
    {
        // RCLCPP_INFO(this->get_logger(), "[0th] = '%d' [10000th] = '%d'", _msg->data[10001], _msg->data[10000]);
        int x = target_nums_[0] * target_nums_[1];
        int y = target_nums_[2] * target_nums_[3];

        // depth는 16uc1 픽셀당 2바이트
        size_t index = (y * _msg->step) + (x * 2);

        uint16_t depth_value = _msg->data[index] + _msg->data[index+1] * 256;
        if (depth_value != 0){
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "x, y: %d, %d => depth: %d", x, y, depth_value);
        }
    }

    void gyro_callback(sensor_msgs::msg::Imu::SharedPtr _msg)
    {
        // RCLCPP_INFO(this->get_logger(), "[0th] = '%d' [10000th] = '%d'", _msg->data[10001], _msg->data[10000]);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "gyro_x: %f, gyro_y: %f, gyro_z: %f", _msg->angular_velocity.x, _msg->angular_velocity.y, _msg->angular_velocity.z);
    }

    void num_callback(std_msgs::msg::Int32MultiArray::SharedPtr _msg)
    {
        target_nums_[0] = _msg->data[0];
        target_nums_[1] = _msg->data[1];
        target_nums_[2] = _msg->data[2];
        target_nums_[3] = _msg->data[3];
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr num_sub_;
    std::vector<int> target_nums_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cw_sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}