#include <iostream>
#include <pqxx/pqxx>
#include <vector>
#include <chrono>

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

    // Function to display trade information
    void print() const {
        std::cout << "Date: " << trade_date
                  << ", Time: " << trade_time
                  << ", Timestamp: " << execution_timestamp
                  << ", Trader ID: " << trader_id
                  << ", Symbol: " << asset_symbol
                  << ", Quantity: " << quantity
                  << ", Price: " << price << std::endl;
    }
};

int main() {
    try {
        pqxx::connection C("dbname=mydatabase user=harrison password=huhaoren hostaddr=127.0.0.1 port=5432");
        if (C.is_open()) {
            std::cout << "Opened database successfully: " << C.dbname() << std::endl;
        } else {
            std::cerr << "Can't open database" << std::endl;
            return 1;
        }

        auto start = std::chrono::high_resolution_clock::now();

        // Start a non-transactional session to execute a read query
        pqxx::nontransaction N(C);
        pqxx::result R = N.exec("SELECT trade_date, trade_time, execution_timestamp, trader_id, asset_symbol, quantity, price FROM trades");

        std::vector<Trade> trades;
        for (auto row : R) {
            trades.emplace_back(
                row[0].as<std::string>(),
                row[1].as<std::string>(),
                row[2].as<int64_t>(),
                row[3].as<int>(),
                row[4].as<std::string>(),
                row[5].as<int>(),
                row[6].as<double>()
            );
        }
        
        for (const auto& trade : trades) {
            trade.print();  // Call the print function for each trade
        }


        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Database read time: " << elapsed.count() << " seconds." << std::endl;


        C.disconnect();
    } catch (const std::exception &e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
