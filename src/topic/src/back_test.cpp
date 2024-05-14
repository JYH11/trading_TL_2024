#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <arrow/array.h>
#include <arrow/builder.h>
#include <arrow/type.h>
#include <memory>
#include <arrow/api.h>
#include <parquet/arrow/writer.h>
#include "interfaces/msg/template_info.hpp"

double account = 0;


class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "backtest node is running.");
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
        std::string example_symbol = msgs->symbol;
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

        double number1 = example_bid_size*example_bid_price;
        double number2 = example_ask_size*example_ask_price;
        double number3 = number2 - number1;

        if(number3 > 0){
            account -= number2;
        }else{
            account += number1;
        }
    }
    
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>("back_test");
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout<<
    return 0;
}