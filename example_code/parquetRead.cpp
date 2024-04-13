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

    // Define variables to read data
    std::string name;
    float price;
    uint32_t quantity;

    // Read data
    while (!os.eof()) {
        os >> name >> price >> quantity >> parquet::EndRow;
        std::cout << "Name: " << name << ", Price: " << price << ", Quantity: " << quantity << std::endl;
    }

    return 0;
}
