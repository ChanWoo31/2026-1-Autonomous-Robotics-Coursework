#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;

class rgbd_read : public rclcpp::Node
{
    public:
    rgbd_read() : Node("rgbd__")
    {
        auto default_qos = rclcpp::SensorDataQoS();
        rgbd_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw", default_qos,
        std::bind(&rgbd_read::rgbd_callback, this, _1));
    }
    private:
    void rgbd_callback(const sensor_msgs::msg::Image::SharedPtr _msg) const
    {
        int x = 240;
        int y = 424;
        size_t index = (y * _msg->step) + (x * 2);

        uint16_t depth_value = _msg->data[index] + _msg->data[index+1] * 256;
        RCLCPP_INFO(this->get_logger(), "[depth] = '%d'", depth_value);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbd_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<rgbd_read>();
    //RCLCPP_INFO(a->get_logger(), "GOOD!!");
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}