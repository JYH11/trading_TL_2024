
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <arrow/result.h>
using std::string;
struct Trade {
    string symbol;       
    double bid_size;
    double bid_price;
    double ask_size;
    double ask_price;

    Trade(string symbol_, double bid_size_, double bid_price_, double ask_size_, double ask_price_) :
        symbol(symbol_), bid_size(bid_size_), bid_price(bid_price_),
        ask_size(ask_size_), ask_price(ask_price_){}
};
/*
class DataWriter {
public:
    DataWriter() {
        setupParquetWriter();
        writer_thread_ = std::thread(&DataWriter::writeDataToDisk, this);
    }

    ~DataWriter() {
        if (file_writer_) {
            file_writer_->Close();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_shutting_down_ = true;
            cond_var_.notify_one();
        }
        writer_thread_.join();
    }

    void setupParquetWriter() {
    string path_to_file = "trades.parquet";
    auto schema = std::static_pointer_cast<parquet::schema::GroupNode>(
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
    PARQUET_THROW_NOT_OK(arrow::io::FileOutputStream::Open(path_to_file, false, &outfile));
    parquet::WriterProperties::Builder builder;
    file_writer_ = parquet::ParquetFileWriter::Open(outfile, schema, builder.build());
    os_ = std::make_unique<parquet::StreamWriter>(std::move(file_writer_));
}


    void queueData(const Trade& trade) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.push(trade);
        cond_var_.notify_one();
    }

private:
    std::thread writer_thread_;
    std::shared_ptr<arrow::io::FileOutputStream> outfile;
    std::shared_ptr<parquet::ParquetFileWriter> file_writer_;
    std::queue<Trade> buffer_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
    bool is_shutting_down_ = false;
    std::unique_ptr<parquet::StreamWriter> os_;

    void writeDataToDisk() {
        while (true) {
            std::unique_lock<std::mutex> lock(mutex_);
            cond_var_.wait(lock, [this]() { return !buffer_.empty() || is_shutting_down_; });
            if (is_shutting_down_ && buffer_.empty()) {
                return;
            }
            Trade trade = buffer_.front();
            buffer_.pop();
            lock.unlock();

            saveTrade(trade);
        }
    }

    void saveTrade(const Trade& trade) {
        *os_ << trade.symbol << trade.bid_size << trade.bid_price
             << trade.ask_size << trade.ask_price << parquet::EndRow;
    }
};
*/