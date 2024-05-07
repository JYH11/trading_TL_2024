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
};

std::vector<Trade> getTrades() {
    std::vector<Trade> trades;
    for (int i = 0; i < 500; ++i) {  // Generating 500 trades
        trades.emplace_back("2023-10-01", "10:00:00", 1664614800000, 2, "MSFT", 150, 250.75);
    }
    return trades;
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();

    try {
        pqxx::connection C("dbname=mydatabase user=harrison password=huhaoren hostaddr=127.0.0.1 port=5432");
        if (C.is_open()) {
            std::cout << "Opened database successfully: " << C.dbname() << std::endl;
        } else {
            std::cout << "Can't open database" << std::endl;
            return 1;
        }

        std::vector<Trade> trades = getTrades();
        pqxx::work W(C);

        for (const auto& trade : trades) {
            W.exec0("INSERT INTO trades (trade_date, trade_time, execution_timestamp, trader_id, asset_symbol, quantity, price) VALUES "
                    "(" + W.quote(trade.trade_date) + ", " + W.quote(trade.trade_time) + ", " +
                    W.quote(trade.execution_timestamp) + ", " + W.quote(trade.trader_id) + ", " +
                    W.quote(trade.asset_symbol) + ", " + W.quote(trade.quantity) + ", " +
                    W.quote(trade.price) + ")");
        }

        W.commit();

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Database write time: " << elapsed.count() << " seconds." << std::endl;

        C.disconnect();
    } catch (const std::exception &e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
