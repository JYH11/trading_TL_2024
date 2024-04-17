#include "arrow/io/api.h"
#include "parquet/arrow/schema.h"
#include "parquet/arrow/writer.h"
#include "parquet/stream_writer.h"
#include <arrow/api.h>
#include <iostream>
#include <memory>

/*
TODO: I just used string int64_t int and double,
      it may cause problems using type of time like TIME_MILLIS
      But in the future, we may fix this bug.
*/

// Define a Trade structure
struct Trade {
    std::string trade_date;       
    std::string trade_time;          
    int64_t execution_timestamp;     
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;

    // constructor 
    Trade(std::string date, std::string time, int64_t timestamp, int trader, std::string symbol, int qty, double pr) :
        trade_date(date), trade_time(time), execution_timestamp(timestamp),
        trader_id(trader), asset_symbol(symbol), quantity(qty), price(pr) {}
};

// Function to simulate getting trade data
std::vector<Trade> getTrades() {
    return {
        {"2023-10-01", "09:00:00", 1664611200000, 1, "AAPL", 100, 150.25},
        {"2023-10-01", "10:00:00", 1664614800000, 2, "MSFT", 150, 250.75},
        // Additional trades can be added here
    };
}

int main() {
    // parquet file path
    std::string path_to_file = "test.parquet";

    // Define schema using Parquet schema primitives directly
    std::shared_ptr<parquet::schema::GroupNode> schema = std::static_pointer_cast<parquet::schema::GroupNode>(
    parquet::schema::GroupNode::Make(
        "trading_record",
        parquet::Repetition::REQUIRED, {
            parquet::schema::PrimitiveNode::Make("trade_date", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("trade_time", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("execution_timestamp", parquet::Repetition::REQUIRED, parquet::Type::INT64, parquet::ConvertedType::INT_64),
            parquet::schema::PrimitiveNode::Make("trader_id", parquet::Repetition::REQUIRED, parquet::Type::INT32, parquet::ConvertedType::INT_32),
            parquet::schema::PrimitiveNode::Make("asset_symbol", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("quantity", parquet::Repetition::REQUIRED, parquet::Type::INT32, parquet::ConvertedType::INT_32),
            parquet::schema::PrimitiveNode::Make("price", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE)
        }));

    //Open file output stream
    std::shared_ptr<arrow::io::FileOutputStream> outfile;
    PARQUET_ASSIGN_OR_THROW(outfile, arrow::io::FileOutputStream::Open(path_to_file));

    //Create Parquet writer and StreamWriter
    parquet::WriterProperties::Builder builder;
    parquet::StreamWriter os{parquet::ParquetFileWriter::Open(outfile, schema, builder.build())};

    // Get data and write
    for (const auto& trade : getTrades()) {
        os << trade.trade_date << trade.trade_time << trade.execution_timestamp
           << trade.trader_id << trade.asset_symbol << trade.quantity << trade.price << parquet::EndRow;
    }

    return 0;
}
