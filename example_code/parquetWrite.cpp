#include "arrow/io/api.h"
#include "parquet/arrow/schema.h"
#include "parquet/arrow/writer.h"
#include "parquet/stream_writer.h"
#include <arrow/api.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <vector>

struct Trade {
    std::string trade_date;       
    std::string trade_time;          
    int64_t execution_timestamp;     
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;

    Trade(std::string date, std::string time, int64_t timestamp, int trader, std::string symbol, int qty, double pr) :
        trade_date(date), trade_time(time), execution_timestamp(timestamp),
        trader_id(trader), asset_symbol(symbol), quantity(qty), price(pr) {}
};

std::vector<Trade> getTrades() {
    std::vector<Trade> trades;
    for (int i = 0; i < 2000; ++i) { 
        trades.emplace_back("2023-10-01", "10:00:00", 1664614800000, 2, "MSFT", 150, 250.75);
    }
    return trades;
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();

    std::string path_to_file = "large_test.parquet";
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

    std::shared_ptr<arrow::io::FileOutputStream> outfile;
    PARQUET_THROW_NOT_OK(arrow::io::FileOutputStream::Open(path_to_file).Value(&outfile));
    parquet::WriterProperties::Builder builder;
    std::unique_ptr<parquet::ParquetFileWriter> file_writer = parquet::ParquetFileWriter::Open(outfile, schema, builder.build());
    parquet::StreamWriter os(std::move(file_writer));

    for (const auto& trade : getTrades()) {
        os << trade.trade_date << trade.trade_time << trade.execution_timestamp
           << trade.trader_id << trade.asset_symbol << trade.quantity << trade.price << parquet::EndRow;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Parquet write time: " << elapsed.count() << " seconds." << std::endl;

    return 0;
}
