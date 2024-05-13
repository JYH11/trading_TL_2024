#include <cstdlib>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ccapi_cpp/ccapi_session.h"
#include "interfaces/msg/template_info.hpp"

using namespace std::chrono_literals;

using ::ccapi::Event;
using ::ccapi::EventDispatcher;
using ::ccapi::Session;
using ::ccapi::SessionConfigs;
using ::ccapi::SessionOptions;
using ::ccapi::Subscription;
using ::ccapi::toString;

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
        // Create message
        interfaces::msg::TemplateInfo info;
    
        SessionOptions sessionOptions;
        SessionConfigs sessionConfigs;
        Session session(sessionOptions, sessionConfigs);
        // Subscription to Binance
        Subscription binanceSubscription("binance", "BTCUSDT", "MARKET_DEPTH");
        session.subscribe(binanceSubscription);

        // Subscription to BitMEX
        Subscription bitmexSubscription("bitmex", "XBTUSD", "MARKET_DEPTH");
        session.subscribe(bitmexSubscription);

        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::vector<Event> eventList = session.getEventQueue().purge();

        for (const auto& event : eventList) {
            std::string pub = toString(event);
            info.symbol = pub;
            // make an announcement
            publisher_->publish(info);               
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", pub.c_str());
        
        }
        session.stop();
    
        std::cout << "Bye" << std::endl;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PublisherNode>("data_pull");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}