#include "arrow/io/api.h"
#include "parquet/arrow/reader.h"
#include "parquet/stream_reader.h"
#include <iostream>
#include <memory>

int main() {
    // parquet file path
    std::string path_to_file = "test.parquet";

    // Open file input stream
    std::shared_ptr<arrow::io::ReadableFile> infile;
    PARQUET_ASSIGN_OR_THROW(infile, arrow::io::ReadableFile::Open("./test.parquet"));

    // Create Parquet reader
    auto parquet_reader = parquet::ParquetFileReader::Open(infile);

    // Create Parquet StreamReader
    parquet::StreamReader os(std::move(parquet_reader)); 

    std::string trade_date;           
    std::string trade_time;           
    int64_t execution_timestamp;      
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;

    // Read data
    while (!os.eof()) {
        os >> trade_date >> trade_time >> execution_timestamp
        >> trader_id >> asset_symbol >> quantity >> price >> parquet::EndRow;

        std::cout << "trade_date: " << trade_date << ", trade_time: " << trade_time << ", execution_timestamp: " 
        << execution_timestamp << ", trader_id:" <<trader_id<< ", asset_symbol:" << asset_symbol
        << ", quantity:" << quantity << ", price:"<<price<< std::endl;
    }

    return 0;
}
