#pragma once
#include "Arduino.h"
#include <map>

enum httpd_method_t { HTTP_GET, HTTP_POST };
constexpr int HTTPD_RESP_USE_STRLEN = -1;
struct httpd_req_t
{
  const char* uri = nullptr;
  httpd_method_t method = HTTP_GET;
  void* user_ctx = nullptr;
  size_t content_len = 0;
  std::string body;
  size_t offset = 0;
  std::map<std::string, std::string> headers;
  std::map<std::string, std::string> responseHeaders;
  std::string response;
  std::string status;
  std::string contentType;
};
struct httpd_uri_t
{
  const char* uri = nullptr;
  httpd_method_t method = HTTP_GET;
  esp_err_t (*handler)(httpd_req_t*) = nullptr;
  void* user_ctx = nullptr;
};
struct FakeHttpServer { std::vector<httpd_uri_t> routes; };
inline FakeHttpServer fakeHttpServer;
using httpd_handle_t = FakeHttpServer*;
struct httpd_config_t
{
  int max_open_sockets = 0;
  bool lru_purge_enable = false;
  int recv_wait_timeout = 0, send_wait_timeout = 0;
};
#define HTTPD_DEFAULT_CONFIG() httpd_config_t{}
inline esp_err_t httpd_start(httpd_handle_t* server, const httpd_config_t*)
{
  if (Fake::httpFailure)
    return ESP_FAIL;
  fakeHttpServer.routes.clear();
  *server = &fakeHttpServer;
  return ESP_OK;
}
inline esp_err_t httpd_stop(httpd_handle_t) { return ESP_OK; }
inline esp_err_t httpd_register_uri_handler(httpd_handle_t server, const httpd_uri_t* route)
{
  server->routes.push_back(*route);
  return ESP_OK;
}
inline esp_err_t httpd_resp_set_status(httpd_req_t* request, const char* status)
{
  request->status = status;
  return ESP_OK;
}
inline esp_err_t httpd_resp_set_type(httpd_req_t* request, const char* type)
{
  request->contentType = type;
  return ESP_OK;
}
inline esp_err_t httpd_resp_set_hdr(httpd_req_t* request, const char* key, const char* value)
{
  request->responseHeaders[key] = value;
  return ESP_OK;
}
inline esp_err_t httpd_resp_send(httpd_req_t* request, const char* text, int length)
{
  request->response.assign(text, length < 0 ? strlen(text) : size_t(length));
  return ESP_OK;
}
inline size_t httpd_req_get_hdr_value_len(httpd_req_t* request, const char* key)
{
  return request->headers[key].size();
}
inline esp_err_t httpd_req_get_hdr_value_str(httpd_req_t* request, const char* key, char* target, size_t size)
{
  const std::string& value = request->headers[key];
  if (value.size() >= size)
    return ESP_FAIL;
  memcpy(target, value.c_str(), value.size() + 1);
  return ESP_OK;
}
inline int httpd_req_recv(httpd_req_t* request, char* target, size_t size)
{
  Fake::now += Fake::httpReceiveDelayMs;
  const size_t count = (std::min)(size, request->body.size() - request->offset);
  memcpy(target, request->body.data() + request->offset, count);
  request->offset += count;
  return int(count);
}
