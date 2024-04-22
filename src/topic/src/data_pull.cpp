#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp> 
#include "interfaces/msg/template_info.hpp"


using namespace std::chrono_literals;

// TODO: Here we need to obtain data from the APIs of various platforms. 
// Generally, the data is in the form of Json files so we need to parse it.

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 2.Create publisher
        publisher_ = this->create_publisher<interfaces::msg::TemplateInfo>("string_msg", 10);

        // Create timer to publish information
        timer_ = this->create_wall_timer(
        500ms,std::bind(&PublisherNode::send_msg, this));
    }

private:

    // 4. declare timer
    rclcpp::TimerBase::SharedPtr timer_;
    // 1. Claim publisher
    rclcpp::Publisher<interfaces::msg::TemplateInfo>::SharedPtr publisher_;
    

   void send_msg()
    { 

        interfaces::msg::TemplateInfo info;
        // Create message
        info.symbol = "XBTUSD";
        info.id = 25585432705;
        info.side = "Sell";
        info.timestamp = "2024-03-24T13:50:14.196Z";
        info.size = 34000;
        info.price = 65479;

        interfaces::msg::TemplateInfo info2;
        info2.symbol = "ASDFG";
        info2.id = 1584;
        info2.side = "sold";
        info2.timestamp = "2024-03-24T88:50:14.196Z";
        info2.size = 987456;
        info2.price = 89522;
        // Log printing
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", info.symbol.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", info2.symbol.c_str());

        // make an announcement
        publisher_->publish(info);
        publisher_->publish(info2);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>("Pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}