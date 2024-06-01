#include <arrow/io/api.h>
#include <parquet/arrow/schema.h>
#include <parquet/arrow/writer.h>
#include <parquet/arrow/reader.h>
#include <parquet/stream_writer.h>
#include <arrow/api.h>
#include <iostream>
#include <vector>
#include <string>

struct Trade {
    std::string symbol;
    double bid_size;
    double bid_price;
    double ask_size;
    double ask_price;

    Trade(std::string symbol_, double bid_size_, double bid_price_, double ask_size_, double ask_price_)
        : symbol(symbol_), bid_size(bid_size_), bid_price(bid_price_),
          ask_size(ask_size_), ask_price(ask_price_) {}
};

class DataWriter {
public:
    DataWriter(const std::string& file_path) : file_path_(file_path) {}

    void writeData(const std::vector<Trade>& trades)
    {
        // Read existing data
        std::vector<Trade> existing_trades = readExistingData();

        // Append new trades to existing trades
        existing_trades.insert(existing_trades.end(), trades.begin(), trades.end());

        // Write combined trades to Parquet file
        writeTradesToFile(existing_trades);
    }

private:
    std::string file_path_;

    std::vector<Trade> readExistingData() {
        std::vector<Trade> trades;
        std::shared_ptr<arrow::io::ReadableFile> infile;
        arrow::Status status = arrow::io::ReadableFile::Open(file_path_, arrow::default_memory_pool()).Value(&infile);
        if (!status.ok()) {
            std::cerr << "No existing file found. Creating a new one." << std::endl;
            return trades;
        }

        std::unique_ptr<parquet::arrow::FileReader> reader;
        parquet::arrow::FileReaderBuilder builder;
        PARQUET_THROW_NOT_OK(builder.Open(infile));
        PARQUET_THROW_NOT_OK(builder.Build(&reader));

        reader->set_use_threads(true);

        std::shared_ptr<arrow::Table> table;
        PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

        auto symbol_array = std::static_pointer_cast<arrow::StringArray>(table->column(0)->chunk(0));
        auto bid_size_array = std::static_pointer_cast<arrow::DoubleArray>(table->column(1)->chunk(0));
        auto bid_price_array = std::static_pointer_cast<arrow::DoubleArray>(table->column(2)->chunk(0));
        auto ask_size_array = std::static_pointer_cast<arrow::DoubleArray>(table->column(3)->chunk(0));
        auto ask_price_array = std::static_pointer_cast<arrow::DoubleArray>(table->column(4)->chunk(0));

        for (int64_t i = 0; i < table->num_rows(); ++i) {
            trades.emplace_back(symbol_array->GetString(i), bid_size_array->Value(i), bid_price_array->Value(i), ask_size_array->Value(i), ask_price_array->Value(i));
        }

        return trades;
    }

    void writeTradesToFile(const std::vector<Trade>& trades) {
        std::shared_ptr<parquet::schema::GroupNode> schema = std::static_pointer_cast<parquet::schema::GroupNode>(
            parquet::schema::GroupNode::Make(
                "trading_record",
                parquet::Repetition::REQUIRED, {
                    parquet::schema::PrimitiveNode::Make("symbol", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
                    parquet::schema::PrimitiveNode::Make("bid_size", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("bid_price", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("ask_size", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("ask_price", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE)
                }));

        std::shared_ptr<arrow::io::FileOutputStream> outfile;
        PARQUET_THROW_NOT_OK(arrow::io::FileOutputStream::Open(file_path_).Value(&outfile));
        parquet::WriterProperties::Builder builder;
        std::unique_ptr<parquet::ParquetFileWriter> file_writer = parquet::ParquetFileWriter::Open(outfile, schema, builder.build());
        parquet::StreamWriter os(std::move(file_writer));

        for (const auto& trade : trades) {
            os << trade.symbol << trade.bid_size << trade.bid_price
               << trade.ask_size << trade.ask_price << parquet::EndRow;
        }
    }
};
