#include "arrow/io/api.h"
#include "parquet/arrow/schema.h"
#include "parquet/arrow/writer.h"
#include "parquet/stream_writer.h"
#include <arrow/api.h>
#include <iostream>
#include <memory>

struct Article {
    std::string name;
    float price;
    uint32_t quantity;

    Article(std::string n, float p, uint32_t q) : name(n), price(p), quantity(q) {}
};

std::vector<Article> getArticles() {
    return {
        {"Apple", 0.50, 10},
        {"Banana", 0.20, 20},
        {"Orange", 0.30, 30}
    };
}

int main() {
    // parquet file path
    std::string path_to_file = "test.parquet";

    // Define schema
    auto int32_type = arrow::int32();
    auto float_type = arrow::float32();
    auto string_type = arrow::utf8();

    std::shared_ptr<parquet::schema::GroupNode> schema = std::static_pointer_cast<parquet::schema::GroupNode>(
    parquet::schema::GroupNode::Make(
        "schema",
        parquet::Repetition::REQUIRED, {
            parquet::schema::PrimitiveNode::Make("name", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("price", parquet::Repetition::REQUIRED, parquet::Type::FLOAT, parquet::ConvertedType::NONE),
            // Specifies that the conversion type of the 'quantity' column is UINT_32
            parquet::schema::PrimitiveNode::Make("quantity", parquet::Repetition::REQUIRED, parquet::Type::INT32, parquet::ConvertedType::UINT_32)
        }));


    //Open file output stream
    std::shared_ptr<arrow::io::FileOutputStream> outfile;
    PARQUET_ASSIGN_OR_THROW(outfile, arrow::io::FileOutputStream::Open(path_to_file));

    //Create Parquet writer and StreamWriter
    parquet::WriterProperties::Builder builder;
    parquet::StreamWriter os{parquet::ParquetFileWriter::Open(outfile, schema, builder.build())};

    // Get data and write
    for (const auto& article : getArticles()) {
        os << article.name << article.price << article.quantity << parquet::EndRow;
    }


    return 0;
}
