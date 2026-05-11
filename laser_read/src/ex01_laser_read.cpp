#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class laser_read:public rclcpp::Node
{
    public:
    laser_read():Node("lidar__")
    {
        auto default_qos = rclcpp::SensorDataQoS();
        lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&laser_read::lidar_callback,this,_1)
        );
    }
    private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) const
    {
        RCLCPP_INFO(this->get_logger(), "[0th] = '%f' [100th] = '%f", _msg->ranges[0], _msg->ranges[100]);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<laser_read>();
    RCLCPP_INFO(a->get_logger(), "GOOD!!");
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}