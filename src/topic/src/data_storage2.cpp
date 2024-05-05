#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <arrow/array.h>
#include <arrow/builder.h>
#include <arrow/type.h>
#include <memory>
#include <arrow/api.h>
#include <parquet/arrow/writer.h>
#include "interfaces/msg/template_info.hpp"

// TODO: This is a subscriber node responsible for transmitting to the local database
// The obtained data needs to be selectively stored as parquet files.

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 3. Create a subscriber
        subscription_ = this->create_subscription<interfaces::msg::TemplateInfo>("string_msg", 10,
                                                                          std::bind(&SubscriberNode::sub_callback, this, std::placeholders::_1));
    }

private:
    // 1.Declare subscribers
    rclcpp::Subscription<interfaces::msg::TemplateInfo>::SharedPtr subscription_;
    // 2.Subscriber callback function
    void sub_callback(const interfaces::msg::TemplateInfo::SharedPtr msgs)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving name: %s, id: %ld, side: %s \n                                     timestamp: %s, size: %ld, price: %ld",
                    msgs->symbol.c_str(),
                    msgs->id,
                    msgs->side.c_str(),
                    msgs->timestamp.c_str(),
                    msgs->size,
                    msgs->price);
    }
    
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>("data_storage2");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}