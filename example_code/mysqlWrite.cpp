#include <mysqlx/xdevapi.h>
#include <iostream>
#include <chrono>
#include <vector>

using namespace std;
using namespace mysqlx;

struct Trade {
    std::string trade_date;
    std::string trade_time;
    int64_t execution_timestamp;
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;
};

vector<Trade> getTrades() {
    vector<Trade> trades;
    for (int i = 0; i < 500; ++i) {
        trades.emplace_back("2023-10-01", "10:00:00", 1664614800000, 2, "MSFT", 150, 250.75);
    }
    return trades;
}

int main() {
    try {
        Session session("localhost", 33060, "root", "your_password", "trading_system");
        Schema db = session.getSchema("trading_system");
        Table table = db.getTable("trades");

        auto start = chrono::high_resolution_clock::now();

        // Start a transaction for batch insertion
        session.startTransaction();

        for (const auto& trade : getTrades()) {
            table.insert("trade_date", "trade_time", "execution_timestamp", "trader_id", "asset_symbol", "quantity", "price")
                 .values(trade.trade_date, trade.trade_time, trade.execution_timestamp, trade.trader_id, trade.asset_symbol, trade.quantity, trade.price)
                 .execute();
        }

        // Commit the transaction
        session.commit();

        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = end - start;
        cout << "MySQL insert time: " << elapsed.count() << " seconds." << endl;
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        // If there is an error, rollback the transaction
        session.rollback();
        return 1;
    }
    return 0;
}
