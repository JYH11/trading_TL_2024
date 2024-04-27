#include <cstdlib>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ccapi_cpp/ccapi_session.h"
#include "interfaces/msg/template_info.hpp"

using namespace std::chrono_literals;

namespace ccapi {
Logger* Logger::logger = nullptr;  // This line is needed.
class MyEventHandler : public EventHandler {
 public:
  bool processEvent(const Event& event, Session* session) override {
    std::cout << toString(event) + "\n" << std::endl;
    return true;
  }
};
} /* namespace ccapi */

using ::ccapi::Event;
using ::ccapi::EventDispatcher;
using ::ccapi::MyEventHandler;
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
        interfaces::msg::TemplateInfo info;
        // Create message
        info.symbol = "XBTUSD";
        info.id = 25585432705;
        info.side = "Sell";
        info.timestamp = "2024-03-24T13:50:14.196Z";
        info.size = 34000;
        info.price = 65479;

        interfaces::msg::TemplateInfo info2;

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
    auto node = std::make_shared<PublisherNode>("data_pull");
    std::vector<std::string> modeList = {
            "dispatch_events_to_multiple_threads",
            "handle_events_in_batching_mode",
    };
    if (argc != 2 || std::find(modeList.begin(), modeList.end(), argv[1]) == modeList.end()) {
    std::cerr << "Please provide one command line argument from this list: " + toString(modeList) << std::endl;
        return EXIT_FAILURE;
    }
    std::string mode(argv[1]);
    if (mode == "dispatch_events_to_multiple_threads") {
        SessionOptions sessionOptions;
        SessionConfigs sessionConfigs;
        MyEventHandler eventHandler;
        EventDispatcher eventDispatcher(2);
        Session session(sessionOptions, sessionConfigs, &eventHandler, &eventDispatcher);
        // Subscription to Binance
        Subscription binanceSubscription("binance", "BTCUSDT", "MARKET_DEPTH");
        session.subscribe(binanceSubscription);
        // Subscription to BitMEX
        Subscription bitmexSubscription("bitmex", "XBTUSD", "MARKET_DEPTH");
        session.subscribe(bitmexSubscription);

        std::this_thread::sleep_for(std::chrono::seconds(10));
        session.stop();
        eventDispatcher.stop();
    } else if (mode == "handle_events_in_batching_mode") {
        SessionOptions sessionOptions;
        SessionConfigs sessionConfigs;
        Session session(sessionOptions, sessionConfigs);
        // Subscription to Binance
        Subscription binanceSubscription("binance", "BTCUSDT", "MARKET_DEPTH");
        session.subscribe(binanceSubscription);

        // Subscription to BitMEX
        Subscription bitmexSubscription("bitmex", "XBTUSD", "MARKET_DEPTH");
        session.subscribe(bitmexSubscription);

        std::this_thread::sleep_for(std::chrono::seconds(10));
        std::vector<Event> eventList = session.getEventQueue().purge();
        for (const auto& event : eventList) {
            std::cout << toString(event) + "\n" << std::endl;
        }
        session.stop();
    }
    std::cout << "Bye" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}