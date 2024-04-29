#include <iostream>
#include <chrono>
#include <vector>
#include <unordered_map>
#include "arrow/io/api.h"
#include "parquet/arrow/reader.h"
#include <arrow/table.h>
#include <memory>

// Cache Implementation
std::unordered_map<std::string, std::shared_ptr<arrow::Table>> cache;
// Function to clear cache
void clear_cache() {
    cache.clear();
}

// Function to read data from cache or disk
std::shared_ptr<arrow::Table> read_data(const std::string& file_path, bool use_cache) {
    std::string key = "unique_key_based_on_file_path";

    if (use_cache) {
        auto it = cache.find(key);
        if (it != cache.end()) {
            return it->second;  // Cache hit
        }
    }

    // Read from disk (Simulating reading from Parquet)
    std::shared_ptr<arrow::io::ReadableFile> infile;
    PARQUET_ASSIGN_OR_THROW(infile, arrow::io::ReadableFile::Open(file_path, arrow::default_memory_pool()));

    std::unique_ptr<parquet::arrow::FileReader> reader;
    parquet::arrow::FileReaderBuilder builder;
    PARQUET_THROW_NOT_OK(builder.Open(infile));
    PARQUET_THROW_NOT_OK(builder.Build(&reader));

    std::shared_ptr<arrow::Table> table;
    PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

    if (use_cache) {
        cache[key] = table;  // Add to cache
    }
    return table;
}

// Testing Function
void performance_test(const std::string& file_path) {
    const int num_runs = 5;

    // Test without cache
    auto start_nc = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_runs; ++i) {
        read_data(file_path, false);
    }
    auto end_nc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_nc = end_nc - start_nc;
    std::cout << "No Cache Total Time: " << elapsed_nc.count() << " seconds." << std::endl;

    // Clear cache
    clear_cache();

    // Test with cache
    auto start_c = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_runs; ++i) {
        read_data(file_path, true);
    }
    auto end_c = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_c = end_c - start_c;
    std::cout << "Cache Total Time: " << elapsed_c.count() << " seconds." << std::endl;
}

int main() {
    std::string file_path = "large_test.parquet";
    performance_test(file_path);
    return 0;
}
