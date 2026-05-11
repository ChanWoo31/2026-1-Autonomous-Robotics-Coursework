#include "rclcpp/rclcpp.hpp"
#include "cw_interfaces/srv/add_two_ints.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <memory>
using std::placeholders::_1;
using std::placeholders::_2;

class cw_server_sub : public rclcpp::Node
{
    public:
    int input_number_;
    
    cw_server_sub(int initial_input)
    : Node("cw_server_sub"), input_number_(initial_input)
    {
        server_ = this->create_service<cw_interfaces::srv::AddTwoInts>(
            "add_two_ints", std::bind(&cw_server_sub::handle_service, this, _1, _2)
        );
    }

    private:
    void handle_service(const std::shared_ptr<cw_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<cw_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = (request->a * request->b) + input_number_;

        RCLCPP_INFO(this->get_logger(), "Client 요청 수신: %ld * %ld + %d = %ld",
    request->a, request->b, input_number_, response->sum);
    }
    rclcpp::Service<cw_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int start_num;
    scanf("%d", &start_num);
    
    auto node = std::make_shared<cw_server_sub>(start_num);
    
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
