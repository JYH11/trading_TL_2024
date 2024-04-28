#include "ccapi_cpp/ccapi_session.h"
namespace ccapi {
Logger* Logger::logger = nullptr;  // This line is needed.
class MyEventHandler : public EventHandler {
 public:
  bool processEvent(const Event& event, Session* session) override {
    std::cout << "Recieved an event: " + event.toStringPretty(2,2) << std::endl;
    return true;
  }
};
} /*namespace ccapi*/
using ::ccapi::MyEventHandler;
using ::ccapi::Request;
using ::ccapi::Session;
using ::ccapi::SessionConfigs;
using ::ccapi::SessionOptions;
using ::ccapi::toString;
using ::ccapi::UtilSystem;
int main(int argc, char** argv){
  std::string key = Utilsystem::getEnvAsString("BITMEX_API_KEY");
  if(key.empty()){
    std::corr<< "please set environment variable BITMEX_API_KEY"
    return EXIT_FAILURE;
  }
  std:: string secret= Utilsystem::getEnvAsString("BITMEX_API_SECRET");
  if(key.empty()){
    std::corr<<"please set environment variable BITMEX_API_SECRET"
    return EXIT_FAILURE
  }
  SessionOptions sessionOptions;
  SessionConfigs sessionConfigs;
  MyEventHandler eventHandler;
  Session session(sessionOptions, sessionConfigs, &eventHandler);
  Request request(Request::Operation::CREATE_ORDER,'bitmex','XBTUSD')
  request.appendParam({
    {"SIDE","BUY"}, 
    {"QUANTITY", '0.001'}, 
    {"LIMIT_PRICE",'20000'} 
  })
  session.sendRequest(request)
  std::this_thread::sleep_for(std::chrono::seconds(10));
  session.stop();
  std::cout << "Bye" << std::endl;
  return EXIT_SUCCESS;
  }