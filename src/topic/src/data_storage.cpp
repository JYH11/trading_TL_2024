#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <arrow/api.h>
#include <parquet/arrow/writer.h>
#include "interfaces/msg/template_info.hpp"

//#include "arrow/io/api.h"
#include "parquet/arrow/schema.h"
//#include "parquet/arrow/writer.h"
#include "parquet/stream_writer.h"
#include <iostream>
#include <memory>
#include <chrono>
#include <vector>
#include "dataWrite.h"
// This is a subscriber node responsible for transmitting to the local database
// The obtained data needs to be selectively stored as parquet files.

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(string name) : Node(name)/*, data_writer_(std::make_unique<DataWriter>())*/
    {
        RCLCPP_INFO(this->get_logger(), "data_storage node is running.");
        // 3. Create a subscriber
        subscription_ = this->create_subscription<interfaces::msg::TemplateInfo>("string_msg", 10,
                                                                          std::bind(&SubscriberNode::sub_callback, this, std::placeholders::_1));
    }

private:
    // 1.Declare subscribers
    rclcpp::Subscription<interfaces::msg::TemplateInfo>::SharedPtr subscription_;

    //std::unique_ptr<DataWriter> data_writer_;

    // 2.Subscriber callback function
    void sub_callback(const interfaces::msg::TemplateInfo::SharedPtr msgs)
    {   
        string example_symbol = msgs->symbol;
        double example_bid_size = msgs->bidsize;
        double example_bid_price = msgs->bidprice;
        double example_ask_size = msgs->asksize;
        double example_ask_price = msgs->askprice;

        RCLCPP_INFO(this->get_logger(), "Receiving name: %s, bidsize: %f, bidprice: %f,asksize: %f, askprice: %f",
                    example_symbol.c_str(),
                    example_bid_size,
                    example_bid_price,
                    example_ask_size,
                    example_ask_price);
        Trade trade(example_symbol, example_bid_size, example_bid_price, example_ask_size , example_ask_price);
        //data_writer_->queueData(trade);
        
    }
    
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>("data_storage");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}