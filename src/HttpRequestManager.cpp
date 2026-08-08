#include "HttpRequestManager.h"

String HttpRequestManager::BuildQueryString(const std::vector<std::pair<String, String>>& params) const
{
    if (params.empty())
        return "";

    String queryStream = "?";

    bool first = true;
    for (const auto& [key, value] : params)
    {
        if (!first)
            queryStream += "&";

        queryStream += key + "=" + value;

        first = false;
    }

    return queryStream;
}

HttpResult HttpRequestManager::Get(const String& url, const std::vector<std::pair<String, String>>& params, const std::vector<std::pair<String, String>>& headers) {
    HttpResult result{ false, 0, "", "" };

    const String queryParams = BuildQueryString(params);
    const String fullUrl = url + queryParams;

    http.begin(fullUrl);

    // add headers to request
    for (const auto& header : headers) {
        http.addHeader(header.first, header.second);
    }

    // send request and handle response
    int responseCode = http.GET();
    result.statusCode = responseCode;

    if (responseCode > 0) {
        result.success = true;
        result.response = http.getString();
    }
    else {
        result.success = false;
        result.errorMessage = http.errorToString(responseCode);
        Serial.print("[GET] HTTP Error (");
        Serial.print(responseCode);
        Serial.print("): ");
        Serial.println(result.errorMessage);
    }

    http.end();
    return result;
}

HttpResult HttpRequestManager::Post(const String& url, const String& body, const std::vector<std::pair<String, String>>& headers)
{
    HttpResult result{ false, 0, "", "" };

    http.begin(url);

    // add headers to request
    for (const auto& header : headers) {
        http.addHeader(header.first, header.second);
    }

    // send request and handle response
    int responseCode = http.POST(body);
    result.statusCode = responseCode;

    if (responseCode > 0) {
        result.success = true;
        result.response = http.getString();
    }
    else {
        result.success = false;
        result.errorMessage = http.errorToString(responseCode);
        Serial.print("[POST] HTTP Error (");
        Serial.print(responseCode);
        Serial.print("): ");
        Serial.println(result.errorMessage);
    }

    http.end();
    return result;
}

bool HttpRequestManager::GetJson(const String& url, JsonDocument& doc, int& statusCode, const std::vector<std::pair<String, String>>& params, const std::vector<std::pair<String, String>>& headers)
{
    const String queryParams = BuildQueryString(params);
    const String fullUrl = url + queryParams;

    http.begin(fullUrl);
    http.useHTTP10(true);  // avoids chunked transfer encoding, so getStream() below
                            // yields the raw JSON body ArduinoJson can parse directly

    for (const auto& header : headers) {
        http.addHeader(header.first, header.second);
    }

    statusCode = http.GET();

    if (statusCode != 200) {
        if (statusCode > 0) {
            Serial.print("[GET] HTTP Error (");
            Serial.print(statusCode);
            Serial.println(")");
        } else {
            Serial.print("[GET] HTTP Error (");
            Serial.print(statusCode);
            Serial.print("): ");
            Serial.println(http.errorToString(statusCode));
        }
        http.end();
        return false;
    }

    // parse directly from the response stream instead of buffering the
    // whole body into a String first - avoids holding two large copies
    // of the payload in RAM at once
    DeserializationError err = deserializeJson(doc, http.getStream());
    http.end();

    if (err) {
        Serial.print("[GET] JSON parse error: ");
        Serial.println(err.c_str());
        return false;
    }

    return true;
}
