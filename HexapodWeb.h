#pragma once

#include <Arduino.h>
#include <esp_http_server.h>
#include <esp_random.h>
#include <freertos/queue.h>
#include "HexapodConsole.h"
#include "HexapodWebPage.h"

namespace Hexapod
{
constexpr uint32_t WEB_TIMEOUT_MS = 500;
constexpr uint32_t WEB_REQUEST_MAX_AGE_MS = 500;
constexpr uint32_t AP_RETRY_MS = 5000;

struct WebMotion
{
  float direction = 0;
  float rotation = 0;
  uint32_t receivedMs = 0;
};

class ConsoleOutput : public Print
{
  char* buffer = nullptr;
  size_t capacity = 0;
  size_t used = 0;
public:
  bool truncated = false;

  void capture(char* target, size_t size)
  {
    buffer = target;
    capacity = size;
    used = 0;
    truncated = false;
    if (buffer != nullptr && capacity != 0)
      buffer[0] = '\0';
  }

  size_t write(uint8_t value) override { return write(&value, 1); }

  size_t write(const uint8_t* data, size_t size) override
  {
    Serial.write(data, size);
    if (buffer != nullptr && capacity != 0)
    {
      const size_t remaining = capacity - used - 1;
      const size_t count = size < remaining ? size : remaining;
      memcpy(buffer + used, data, count);
      used += count;
      buffer[used] = '\0';
      truncated |= count != size;
    }
    return size;
  }
};

enum class WebRequestType { Status, Command, Drive };

struct WebRequest
{
  WebRequestType type = WebRequestType::Status;
  uint32_t id = 0;
  uint32_t receivedMs = 0;
  char body[192] = {};
};

struct WebResponse
{
  uint32_t id = 0;
  uint32_t generation = 0;
  int code = 200;
  char body[4096] = {};
};

inline bool parseWebEnvelope(char* body, uint32_t& generation, char*& payload)
{
  char* separator = strchr(body, '\n');
  if (separator == nullptr)
    return false;
  *separator = '\0';
  if (!parseInteger(body, 0, UINT32_MAX, generation))
    return false;
  payload = separator + 1;
  const size_t length = strlen(payload);
  if (length == 0 || length >= sizeof(ConsoleLine::data))
    return false;
  for (const char* cursor = payload; *cursor; ++cursor)
    if ((*cursor < 32 && *cursor != '\t') || *cursor > 126)
      return false;
  return true;
}

inline bool parseWebMotion(char* payload, float& direction, float& rotation)
{
  char* separator = strchr(payload, ' ');
  if (separator == nullptr)
    return false;
  *separator = '\0';
  return parseNumber(payload, -1, 1, direction) &&
      parseNumber(separator + 1, -1, 1, rotation);
}

class WebTransport
{
  httpd_handle_t server = nullptr;
  uint32_t nextId = 0;

  static const char* statusText(int code)
  {
    switch (code)
    {
      case 200: return "200 OK";
      case 400: return "400 Bad Request";
      case 403: return "403 Forbidden";
      case 409: return "409 Conflict";
      case 413: return "413 Content Too Large";
      case 503: return "503 Service Unavailable";
      case 504: return "504 Gateway Timeout";
      default: return "500 Internal Server Error";
    }
  }

  static esp_err_t reply(httpd_req_t* request, int code, const char* body,
                         const char* type = "text/plain; charset=utf-8")
  {
    httpd_resp_set_status(request, statusText(code));
    httpd_resp_set_type(request, type);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(request, "Content-Security-Policy",
        "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
        "connect-src 'self'; frame-ancestors 'none'; form-action 'none'");
    return httpd_resp_send(request, body, HTTPD_RESP_USE_STRLEN);
  }

  static esp_err_t handle(httpd_req_t* request)
  {
    auto& transport = *static_cast<WebTransport*>(request->user_ctx);
    if (strcmp(request->uri, "/") == 0)
      return reply(request, 200, WEB_PAGE, "text/html; charset=utf-8");

    WebRequest message;
    message.id = ++transport.nextId;
    message.receivedMs = millis();
    if (request->method == HTTP_POST)
    {
      char suppliedToken[sizeof(transport.token)] = {};
      if (httpd_req_get_hdr_value_len(request, "X-Hexapod-Token") != strlen(transport.token) ||
          httpd_req_get_hdr_value_str(request, "X-Hexapod-Token", suppliedToken,
                                     sizeof(suppliedToken)) != ESP_OK ||
          strcmp(suppliedToken, transport.token) != 0)
        return reply(request, 403, "ERROR: reload the control page before sending commands.");
      if (request->content_len == 0 || request->content_len >= sizeof(message.body))
        return reply(request, 413, "ERROR: invalid request length.");
      size_t received = 0;
      while (received < request->content_len)
      {
        const int count = httpd_req_recv(request, message.body + received,
                                        request->content_len - received);
        if (count <= 0)
        {
          reply(request, 400, "ERROR: incomplete request.");
          return ESP_FAIL;
        }
        received += size_t(count);
      }
      if (memchr(message.body, '\0', received) != nullptr)
        return reply(request, 400, "ERROR: invalid request bytes.");
      message.type = strcmp(request->uri, "/api/drive") == 0 ?
          WebRequestType::Drive : WebRequestType::Command;
    }
    // Socket reads/writes run on the HTTP task, never on the servo/watchdog loop.
    if (xQueueSend(transport.requests, &message, 0) != pdPASS)
      return reply(request, 503, "ERROR: controller busy; retry explicitly.");
    static WebResponse response;
    const uint32_t started = millis();
    do
    {
      if (xQueueReceive(transport.responses, &response, pdMS_TO_TICKS(100)) == pdTRUE &&
          response.id == message.id)
      {
        char generation[11];
        snprintf(generation, sizeof(generation), "%lu", static_cast<unsigned long>(response.generation));
        httpd_resp_set_hdr(request, "X-Hexapod-Generation", generation);
        return reply(request, response.code, response.body,
            message.type == WebRequestType::Status ? "application/json" : "text/plain; charset=utf-8");
      }
    } while (uint32_t(millis() - started) < 2000);
    return reply(request, 504, "ERROR: controller did not respond; check status before retrying.");
  }

public:
  QueueHandle_t requests = nullptr;
  QueueHandle_t responses = nullptr;
  char token[17] = {};

  bool begin()
  {
    if (server != nullptr)
      return true;
    if (requests == nullptr)
      requests = xQueueCreate(1, sizeof(WebRequest));
    if (responses == nullptr)
      responses = xQueueCreate(1, sizeof(WebResponse));
    if (requests == nullptr || responses == nullptr)
      return false;
    snprintf(token, sizeof(token), "%08lx%08lx",
        static_cast<unsigned long>(esp_random()), static_cast<unsigned long>(esp_random()));
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 2;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 1;
    if (httpd_start(&server, &config) != ESP_OK)
    {
      server = nullptr;
      return false;
    }
    const char* paths[] = {"/", "/api/status", "/api/command", "/api/drive"};
    for (size_t i = 0; i < 4; ++i)
    {
      httpd_uri_t route = {};
      route.uri = paths[i];
      route.method = i < 2 ? HTTP_GET : HTTP_POST;
      route.handler = handle;
      route.user_ctx = this;
      if (httpd_register_uri_handler(server, &route) != ESP_OK)
      {
        httpd_stop(server);
        server = nullptr;
        return false;
      }
    }
    return true;
  }
};
}
