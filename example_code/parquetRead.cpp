#include "arrow/io/api.h"
#include "parquet/arrow/reader.h"
#include "parquet/stream_reader.h"
#include <arrow/table.h>
#include <iostream>
#include <memory>
#include <chrono>

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    // parquet file path
    std::string file_path = "trades.parquet";

    // Create file input stream
    std::shared_ptr<arrow::io::ReadableFile> infile;
    PARQUET_ASSIGN_OR_THROW(infile, arrow::io::ReadableFile::Open(file_path, arrow::default_memory_pool()));

    // Create a Parquet file reader
    std::unique_ptr<parquet::arrow::FileReader> reader;
    parquet::arrow::FileReaderBuilder builder;
    PARQUET_THROW_NOT_OK(builder.Open(infile));
    PARQUET_THROW_NOT_OK(builder.Build(&reader));

    // Read using multiple threads
    reader->set_use_threads(true);

    // Read the entire table
    std::shared_ptr<arrow::Table> table;
    PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

    // Outputs simple information about the table,
    // which can be expanded to more detailed row and column processing
    std::cout << table->ToString() << std::endl;


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Parquet read time: " << elapsed.count() << " seconds." << std::endl;

    return 0;
}
