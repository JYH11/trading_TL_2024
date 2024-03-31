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
/*
using json = nlohmann::json;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
*/
using namespace std::chrono_literals;

// TODO: Here we need to use the Boost library to obtain data from the APIs of various platforms. 
// Generally, the data is in the form of Json files. Here are some code examples with comments

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 2.Create publisher
        publisher_ = this->create_publisher<interfaces::msg::TemplateInfo>("string_msg", 10);

        /*
        // Start a thread to run the IO context
        std::thread([this] { ioc_.run(); }).detach();

        // Connect to WebSocket
        connect_to_websocket();
        */

        // Create timer to publish information
        timer_ = this->create_wall_timer(
        500ms,std::bind(&PublisherNode::send_msg, this));
    }

private:
    /*
    //Necessary statement
    net::io_context ioc_;
    websocket::stream<net::ip::tcp::socket> ws_;
    beast::flat_buffer buffer_;
    */

    // 4. declare timer
    rclcpp::TimerBase::SharedPtr timer_;
    // 1. Claim publisher
    rclcpp::Publisher<interfaces::msg::TemplateInfo>::SharedPtr publisher_;

    /*
    void connect_to_websocket() {
        // Set the target of the WebSocket server
        auto const host = "example.com";
        auto const port = "80";
        auto const target = "/your_target";

        // Resolve hostname
        net::ip::tcp::resolver resolver{ioc_};
        auto const results = resolver.resolve(host, port);

        // Connect to server
        net::connect(ws_.next_layer(), results.begin(), results.end());

        // WebSocket shake hands
        ws_.handshake(host, target);

        // Send a subscription request to the server
        json subscribe_request = {
            {"command", "subscribe"},
            {"channel", "your_channel"}
        };
        ws_.write(net::buffer(subscribe_request.dump()));

        // Asynchronous reading
        ws_.async_read(buffer_, [this](beast::error_code ec, std::size_t) {
            if (!ec) {
                // Process received messages
                auto msg = beast::buffers_to_string(buffer_.data());
                process_message(msg);
                buffer_.consume(buffer_.size());
            }
        });
    }

    void check_for_messages() {
        //May want to consider using 
        //check and handle messages received from WebSocket here
    }
    

    // 3.release news
    void process_message(const std::string& msg)
    { 
        // Parse the message
        auto data = json::parse(msg);

        interfaces::msg::TemplateInfo info;
        // Create message
        info.symbol = data["symbol"];
        info.id = data["id"];
        info.side = data["side"];
        info.timestamp = data["timestamp"];
        info.size = data["size"];
        info.price = data["price"];

        // Log printing
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", info.symbol.c_str());

        // make an announcement
        publisher_->publish(info);
    }
    */
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