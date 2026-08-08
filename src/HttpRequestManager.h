#pragma once

#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>

struct HttpResult {
    bool success;           // Whether the request succeeded
    int statusCode;         // HTTP status code (0 if network error)
    String response;        // Response body (empty on error)
    String errorMessage;    // Error description if success == false
};

class HttpRequestManager
{
private:
    HTTPClient http;

    String BuildQueryString(const std::vector<std::pair<String, String>>& params) const;

public:
    HttpRequestManager() = default;
    ~HttpRequestManager() = default;

    [[nodiscard]] HttpResult Get(const String& url, const std::vector<std::pair<String, String>>& params = {}, const std::vector<std::pair<String, String>>& headers = {});
    [[nodiscard]] HttpResult Post(const String& url, const String& body = "", const std::vector<std::pair<String, String>>& headers = {});

    // Fetches and parses JSON directly from the response stream, avoiding
    // buffering the whole body into a String first (halves peak RAM use
    // for large responses). doc is only populated when this returns true.
    [[nodiscard]] bool GetJson(const String& url, JsonDocument& doc, int& statusCode, const std::vector<std::pair<String, String>>& params = {}, const std::vector<std::pair<String, String>>& headers = {});
};