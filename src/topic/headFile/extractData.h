#include <string>
using std::string;

string extractField(const string& data, const string& field) {
    size_t startPos = data.find(field + "=");
    if (startPos == string::npos) {
        return "";
    }
    startPos += field.length() + 1; // Start after the '='
    size_t endPos = data.find(",", startPos);
    if (endPos == string::npos) { // Handle the last parameter case
        endPos = data.length();
    }
    return data.substr(startPos, endPos - startPos);
}

string getSymbol(const string& data) {
    return extractField(data, "instrument");
}

string getTime(const string& data){
    size_t startPos = data.find("time =");
    if(startPos == string::npos){
        return "";
    }
    startPos += 7; // "time = "
    size_t endPos = data.find("," , startPos);
    if(endPos == string::npos){
        return "";
    }
    return data.substr(startPos , endPos - startPos);
}


double getBidPrice(const string& data) {
    string price = extractField(data, "BID_PRICE");
    return !price.empty() ? stod(price) : 0.0;
}

double getBidSize(const string& data) {
    string size = extractField(data, "BID_SIZE");
    return !size.empty() ? stod(size) : 0.0;
}

double getAskPrice(const string& data) {
    string price = extractField(data, "ASK_PRICE");
    return !price.empty() ? stod(price) : 0.0;
}

double getAskSize(const string& data) {
    string size = extractField(data, "ASK_SIZE");
    return !size.empty() ? stod(size) : 0.0;
}