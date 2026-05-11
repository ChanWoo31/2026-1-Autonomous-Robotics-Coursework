#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cw_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class cw_sub_cli : public rclcpp::Node
{
    public:

    bool is_data_ready = false;
    int received_a = 0;
    int received_b = 0;
    int received_count = 0;

    cw_sub_cli()
    : Node("cw_sub_cli")
    {
        cw_sub_cli_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic_cw01_aaa", 10, std::bind(&cw_sub_cli::cw01_topic_callback, this, _1)
        );

        client_ = this->create_client<cw_interfaces::srv::AddTwoInts>("add_two_ints");
    }
    rclcpp::Client<cw_interfaces::srv::AddTwoInts>::SharedPtr client_;

    private:

    void cw01_topic_callback(std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        received_a = msg->data[0];
        received_b = msg->data[1];
        received_count = msg->data[2] + 1;
        int c = received_a + received_b;
        RCLCPP_INFO(this->get_logger(), "%d data %d + %d = %d", received_count, received_a, received_b, c);

        is_data_ready = true;

        
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cw_sub_cli_;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cw_sub_cli>();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (node->is_data_ready)
        {
            auto request = std::make_shared<cw_interfaces::srv::AddTwoInts::Request>();
            request->a = node->received_a;
            request->b = node->received_b;
            
            auto result = node->client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(node->get_logger(), "결과: %ld", result.get()->sum);
            }

            node->is_data_ready = false;
        }
    }

    rclcpp::shutdown();
    return 0;
}