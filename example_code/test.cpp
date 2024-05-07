#include <gtest/gtest.h>
#include "parquet/arrow/writer.h"
#include <arrow/io/api.h>
#include <fstream>

class ParquetWriteTest : public ::testing::Test {
protected:
    std::string test_file = "test_output.parquet";

    void SetUp() override {
        // setup code here, if needed
    }

    void TearDown() override {
        // Clean up
        std::remove(test_file.c_str());
    }
};

TEST_F(ParquetWriteTest, WriteReadBack) {
    // Write to a Parquet file
    main();  // Assuming main() writes to 'large_test.parquet', modify as needed

    // Read back the contents
    std::shared_ptr<arrow::io::ReadableFile> infile;
    PARQUET_ASSIGN_OR_THROW(infile, arrow::io::ReadableFile::Open(test_file, arrow::default_memory_pool()));
    std::unique_ptr<parquet::arrow::FileReader> reader;
    parquet::arrow::FileReaderBuilder builder;
    ASSERT_OK(builder.Open(infile));
    ASSERT_OK(builder.Build(&reader));
    std::shared_ptr<arrow::Table> table;
    ASSERT_OK(reader->ReadTable(&table));

    // Assert we read back data correctly; this could be more specific based on the expected output
    ASSERT_NE(table, nullptr);
    ASSERT_EQ(table->num_rows(), 500);  // Expected number of rows
}
