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
        std::string file_path = "large_test.parquet";

    // 创建文件输入流
    std::shared_ptr<arrow::io::ReadableFile> infile;
    PARQUET_ASSIGN_OR_THROW(infile, arrow::io::ReadableFile::Open(file_path, arrow::default_memory_pool()));

    // 创建Parquet文件读取器
    std::unique_ptr<parquet::arrow::FileReader> reader;
    parquet::arrow::FileReaderBuilder builder;
    PARQUET_THROW_NOT_OK(builder.Open(infile));
    PARQUET_THROW_NOT_OK(builder.Build(&reader));

    // 使用多线程读取
    reader->set_use_threads(true);

    // 读取整个表
    std::shared_ptr<arrow::Table> table;
    PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

    // 输出表的简单信息，可以扩展为更详细的行列处理
    std::cout << table->ToString() << std::endl;


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Parquet read time: " << elapsed.count() << " seconds." << std::endl;

    return 0;
}
