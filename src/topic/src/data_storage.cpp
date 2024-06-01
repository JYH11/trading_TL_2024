#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include "interfaces/msg/template_info.hpp"
#include "../headFile/dataWrite.h"
#include <queue>
#include <mutex>
#include <vector>
#include <condition_variable>

using namespace std::chrono_literals;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(std::string name) : Node(name), data_writer_("trades.parquet")
    {
        RCLCPP_INFO(this->get_logger(), "data_storage node is running.");
        // Create a subscriber
        subscription_ = this->create_subscription<interfaces::msg::TemplateInfo>("string_msg", 10,
            std::bind(&SubscriberNode::sub_callback, this, std::placeholders::_1));

        // Create a timer to periodically write data
        timer_ = this->create_wall_timer(1s, std::bind(&SubscriberNode::write_data, this));
    }

private:
    rclcpp::Subscription<interfaces::msg::TemplateInfo>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    DataWriter data_writer_;
    std::queue<Trade> trade_queue_;
    std::mutex queue_mutex_;

    void sub_callback(const interfaces::msg::TemplateInfo::SharedPtr msgs)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        trade_queue_.emplace(msgs->symbol, msgs->bidsize, msgs->bidprice, msgs->asksize, msgs->askprice);
        RCLCPP_INFO(this->get_logger(), "Receiving name: %s, bidsize: %f, bidprice: %f, ask_size: %f, ask_price: %f",
                    msgs->symbol.c_str(),
                    msgs->bidsize,
                    msgs->bidprice,
                    msgs->asksize,
                    msgs->askprice);
    }

    void write_data()
    {
        std::vector<Trade> trades;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            while (!trade_queue_.empty())
            {
                trades.push_back(trade_queue_.front());
                trade_queue_.pop();
            }
        }
        if (!trades.empty())
        {
            data_writer_.writeData(trades);
            RCLCPP_INFO(this->get_logger(), "Wrote %zu trades to Parquet file.", trades.size());
        }
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
