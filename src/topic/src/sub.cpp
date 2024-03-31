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
// There is a commented out example, just an example

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
    /*
    bool should_write_to_parquet() {
        // Example: Write when 100 records are reached
        return id_builder.length() >= 100;
    }
    std::vector<std::shared_ptr<arrow::Array>> data_;
    arrow::Int64Builder id_builder;
    */

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
    /*
    void sub_callback(const interfaces::msg::TemplateInfo::SharedPtr& info) {
        // Convert info data to Arrow Arrays and add to data_
        add_to_data(*info);
        
        if (should_write_to_parquet()) {
            write_to_parquet();
            data_.clear();  // Clear old data
        }
    }
    */

    /*
    void write_to_parquet() {
        // Build Arrow table and write to Parquet
        auto schema = arrow::schema({
        arrow::field("symbol", arrow::utf8()), 
        arrow::field("id", arrow::int64()),       
        arrow::field("side", arrow::utf8()),      
        arrow::field("timestamp", arrow::utf8()), 
        arrow::field("size", arrow::int64()),    
        arrow::field("price", arrow::int64())     
        });
        auto table = arrow::Table::Make(schema, data_);

        // write file
        std::shared_ptr<arrow::io::FileOutputStream> outfile;
        PARQUET_ASSIGN_OR_THROW(outfile, arrow::io::FileOutputStream::Open("output.parquet"));
        PARQUET_THROW_NOT_OK(parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), outfile, 1024));
    }

    void add_to_data(const interfaces::msg::TemplateInfo& info) {
    static arrow::StringBuilder symbol_builder;
    static arrow::Int64Builder id_builder;
    static arrow::StringBuilder side_builder;
    static arrow::StringBuilder timestamp_builder;
    static arrow::Int64Builder size_builder;
    static arrow::Int64Builder price_builder;

    // Append data to each builder
    PARQUET_THROW_NOT_OK(symbol_builder.Append(info.symbol));
    PARQUET_THROW_NOT_OK(id_builder.Append(info.id));
    PARQUET_THROW_NOT_OK(side_builder.Append(info.side));
    PARQUET_THROW_NOT_OK(timestamp_builder.Append(info.timestamp));
    PARQUET_THROW_NOT_OK(size_builder.Append(info.size));
    PARQUET_THROW_NOT_OK(price_builder.Append(info.price));

    // If a certain amount of data is reached, the build is completed and builders are reset
    if (should_write_to_parquet()) {
        std::shared_ptr<arrow::Array> symbol_array, id_array, side_array, timestamp_array, size_array, price_array;
        PARQUET_THROW_NOT_OK(symbol_builder.Finish(&symbol_array));
        PARQUET_THROW_NOT_OK(id_builder.Finish(&id_array));
        PARQUET_THROW_NOT_OK(side_builder.Finish(&side_array));
        PARQUET_THROW_NOT_OK(timestamp_builder.Finish(&timestamp_array));
        PARQUET_THROW_NOT_OK(size_builder.Finish(&size_array));
        PARQUET_THROW_NOT_OK(price_builder.Finish(&price_array));

        // Create Arrow table
        auto schema = arrow::schema({
            arrow::field("symbol", arrow::utf8()),
            arrow::field("id", arrow::int64()),
            arrow::field("side", arrow::utf8()),
            arrow::field("timestamp", arrow::utf8()),
            arrow::field("size", arrow::int64()),
            arrow::field("price", arrow::int64())
        });
        std::vector<std::shared_ptr<arrow::Array>> columns = {symbol_array, id_array, side_array, timestamp_array, size_array, price_array};
        auto table = arrow::Table::Make(schema, columns);

        // Write to Parquet file
        write_to_parquet(table);

        // Reset builders
        symbol_builder.Reset();
        id_builder.Reset();
        side_builder.Reset();
        timestamp_builder.Reset();
        size_builder.Reset();
        price_builder.Reset();
    }
    
    }
    */
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>("Sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}