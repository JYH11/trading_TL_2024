#include "ccapi_cpp/ccapi_session.h"
#include <regex>
#include <vector>
#include <map>

namespace ccapi {
    Logger* Logger::logger = nullptr;  // This line is needed.

    class MyEventHandler : public EventHandler {
    public:
        bool processEvent(const Event& event, Session* session) override {
            std::cout << toString(event) + "\n" << std::endl;

            std::string eventString = toString(event);
            std::string time;
            std::string timeReceived;
            std::string bidPrice;
            std::string bidSize;
            std::string askPrice;
            std::string askSize;

            std::regex timePattern("time = (\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}\\.\\d{9}Z)");
            std::regex timeReceivedPattern("timeReceived = (\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}\\.\\d{9}Z)");
            std::regex bidPricePattern("BID_PRICE=(\\d+\\.\\d+)");
            std::regex bidSizePattern("BID_SIZE=(\\d+\\.\\d+)");
            std::regex askPricePattern("ASK_PRICE=(\\d+\\.\\d+)");
            std::regex askSizePattern("ASK_SIZE=(\\d+\\.\\d+)");

            std::smatch matches;

            if (std::regex_search(eventString, matches, timePattern)) {
                time = matches[1].str();
            }

            if (std::regex_search(eventString, matches, timeReceivedPattern)) {
                timeReceived = matches[1].str();
            }

            if (std::regex_search(eventString, matches, bidPricePattern)) {
                bidPrice = matches[1].str();
            }

            if (std::regex_search(eventString, matches, bidSizePattern)) {
                bidSize = matches[1].str();
            }

            if (std::regex_search(eventString, matches, askPricePattern)) {
                askPrice = matches[1].str();
            }

            if (std::regex_search(eventString, matches, askSizePattern)) {
                askSize = matches[1].str();
            }

            std::cout << "Time: " << time << std::endl;
            std::cout << "Time Received: " << timeReceived << std::endl;
            std::cout << "Bid Price: " << bidPrice << std::endl;
            std::cout << "Bid Size: " << bidSize << std::endl;
            std::cout << "Ask Price: " << askPrice << std::endl;
            std::cout << "Ask Size: " << askSize << std::endl;

            return true;
        }
    };
}  /* namespace ccapi */
using ::ccapi::Event;
using ::ccapi::EventDispatcher;
using ::ccapi::MyEventHandler;
using ::ccapi::Session;
using ::ccapi::SessionConfigs;
using ::ccapi::SessionOptions;
using ::ccapi::Subscription;
using ::ccapi::toString;
int main(int argc, char** argv) {
  std::vector<std::string> modeList = {
      "dispatch_events_to_multiple_threads",
      "handle_events_in_batching_mode",
  };
  if (argc != 2 || std::find(modeList.begin(), modeList.end(), argv[1]) == modeList.end()) {
    std::cerr << "Please provide one command line argument from this list: " + toString(modeList) << std::endl;
    return EXIT_FAILURE;
  }
  std::string mode(argv[1]);
  if (mode == "dispatch_events_to_multiple_threads") {
    SessionOptions sessionOptions;
    SessionConfigs sessionConfigs;
    MyEventHandler eventHandler;
    EventDispatcher eventDispatcher(2);
    Session session(sessionOptions, sessionConfigs, &eventHandler, &eventDispatcher);
    // Subscription to Binance
    Subscription binanceSubscription("binance", "BTCUSDT", "MARKET_DEPTH","MARKET_DEPTH_RETURN_UPDATE=1&MARKET_DEPTH_MAX=10");
    session.subscribe(binanceSubscription);
    // Subscription to BitMEX
    Subscription bitmexSubscription("bitmex", "XBTUSD", "MARKET_DEPTH", "MARKET_DEPTH_RETURN_UPDATE=1&MARKET_DEPTH_MAX=10");
    session.subscribe(bitmexSubscription);
    //Subscription subscription("binance", "BTCUSDT", "MARKET_DEPTH");
    //Subscription subscription("coinbase", "BTC-USD", "MARKET_DEPTH");
    //session.subscribe(subscription);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    session.stop();
    eventDispatcher.stop();
  } else if (mode == "handle_events_in_batching_mode") {
    SessionOptions sessionOptions;
    SessionConfigs sessionConfigs;
    Session session(sessionOptions, sessionConfigs);
    // Subscription to Binance
    Subscription binanceSubscription("binance", "BTCUSDT", "MARKET_DEPTH","MARKET_DEPTH_RETURN_UPDATE=1&MARKET_DEPTH_MAX=10");
    session.subscribe(binanceSubscription);

    // Subscription to BitMEX
    Subscription bitmexSubscription("bitmex", "XBTUSD", "MARKET_DEPTH","MARKET_DEPTH_RETURN_UPDATE=1&MARKET_DEPTH_MAX=10");
    session.subscribe(bitmexSubscription);

    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::vector<Event> eventList = session.getEventQueue().purge();
    for (const auto& event : eventList) {
      std::cout << toString(event) + "\n" << std::endl;
    }
    session.stop();
  }
  std::cout << "Bye" << std::endl;
  return EXIT_SUCCESS;
}
