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

std::string extractField(const std::string& data, const std::string& field) {
    size_t startPos = data.find(field + "=");
    if (startPos == std::string::npos) {
        return "";
    }
    startPos += field.length() + 1; // Start after the '='
    size_t endPos = data.find(",", startPos);
    if (endPos == std::string::npos) { // Handle the last parameter case
        endPos = data.length();
    }
    return data.substr(startPos, endPos - startPos);
}

std::string getSymbol(const std::string& data) {
    return extractField(data, "instrument");
}
/*
std::string getTime(const std::string& data){
    size_t startPos = data.find("time" + " =");
    if(startPos == std::string::npos){
        return "";
    }
    startPos += 7 // "time = "
    size_t endPos = data.find("," , startPos);
    if(endPos == std::string::npos){
        return "";
    }
    return data.substr(startPos , endPos - startPos);
}
*/

double getBidPrice(const std::string& data) {
    std::string price = extractField(data, "BID_PRICE");
    return !price.empty() ? std::stod(price) : 0.0;
}

double getBidSize(const std::string& data) {
    std::string size = extractField(data, "BID_SIZE");
    return !size.empty() ? std::stod(size) : 0.0;
}

double getAskPrice(const std::string& data) {
    std::string price = extractField(data, "ASK_PRICE");
    return !price.empty() ? std::stod(price) : 0.0;
}

double getAskSize(const std::string& data) {
    std::string size = extractField(data, "ASK_SIZE");
    return !size.empty() ? std::stod(size) : 0.0;
}


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
        // Subscription bitmexSubscription("bitmex", "BTCUSD", "MARKET_DEPTH");
        // session.subscribe(bitmexSubscription);

        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::vector<Event> eventList = session.getEventQueue().purge();

        for (const auto& event : eventList) {
            std::string data = toString(event);

            info.symbol = "BTCUSDT";
            info.bidsize = getBidSize(data);
            info.bidprice = getBidPrice(data);
            info.asksize = getAskSize(data);
            info.askprice = getAskPrice(data);

            // make an announcement
            publisher_->publish(info);               
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", data.c_str());
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