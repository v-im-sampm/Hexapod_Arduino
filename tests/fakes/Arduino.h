#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
constexpr int HIGH = 1, LOW = 0, OUTPUT = 3, INPUT_PULLUP = 5, WIFI_STA = 1, WIFI_AP_STA = 3;
constexpr int ESP_OK = 0, WIFI_PS_NONE = 0, WIFI_SECOND_CHAN_NONE = 0;
constexpr int pdPASS = 1, pdTRUE = 1, pdFALSE = 0;
constexpr int ESP_FAIL = -1;
using esp_err_t = int;
#define PROGMEM
#define pdMS_TO_TICKS(value) (value)

namespace Fake
{
inline uint32_t now = 0;
inline int outputEnable = HIGH;
inline int oeMode = -1, oeLatch = LOW;
inline unsigned unconfiguredGpioWrites = 0;
inline std::vector<int> oeStartupLevels;
inline bool boardsPresent = true, writeFailure = false, radioFailure = false;
inline int writesUntilFailure = -1;
inline bool storageFailure = false, readFailure = false;
inline std::vector<uint8_t> saved;
inline unsigned saves = 0;
inline uint16_t pwm[2][16] = {};
inline uint16_t pwmOn[2][16] = {};
inline uint16_t pwmOff[2][16] = {};
struct PwmWrite
{
  uint32_t atMs;
  size_t board;
  uint8_t channel;
  uint16_t on, off;
  int outputEnable;
};
inline std::vector<PwmWrite> pwmWrites;
inline std::vector<int> radioOutputEnable;
inline std::vector<unsigned> radioActiveChannels;
inline std::vector<int> driverOutputEnable;
inline uint8_t channel = 1;
inline bool apStarted = false, apFailure = false, apStopFailure = false, httpFailure = false;
inline int wifiMode = WIFI_STA;
inline std::string apName;
inline void (*waitHook)() = nullptr;
inline uint32_t httpReceiveDelayMs = 0;
}

inline uint32_t millis() { return Fake::now; }
inline void delay(uint32_t ms) { Fake::now += ms; }
inline void pinMode(int pin, int mode)
{
  if (pin != 23)
    return;
  Fake::oeMode = mode;
  Fake::outputEnable = mode == INPUT_PULLUP ? HIGH : Fake::oeLatch;
  if (mode == OUTPUT)
    Fake::oeStartupLevels.push_back(Fake::outputEnable);
}
inline void digitalWrite(int pin, int value)
{
  if (pin != 23)
    return;
  if (Fake::oeMode == -1)
  {
    ++Fake::unconfiguredGpioWrites;
    return;
  }
  Fake::oeLatch = value;
  if (Fake::oeMode == OUTPUT)
    Fake::outputEnable = value;
}
inline int digitalRead(int) { return Fake::outputEnable; }

class Print
{
public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t value) = 0;
  virtual size_t write(const uint8_t* data, size_t size)
  {
    for (size_t i = 0; i < size; ++i)
      write(data[i]);
    return size;
  }
  void println(const char* text)
  {
    write(reinterpret_cast<const uint8_t*>(text), strlen(text));
    write(uint8_t('\n'));
  }
  void printf(const char* format, ...)
  {
    char buffer[4096];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);
    write(reinterpret_cast<const uint8_t*>(buffer), strlen(buffer));
  }
};

struct FakeSerial : Print
{
  std::deque<char> input;
  std::string output;
  void begin(int) {}
  int available() const { return int(input.size()); }
  int read() { const char value = input.front(); input.pop_front(); return value; }
  size_t write(uint8_t value) override { output += char(value); return 1; }
  size_t write(const uint8_t* data, size_t size) override
  {
    output.append(reinterpret_cast<const char*>(data), size);
    return size;
  }
};
inline FakeSerial Serial;

struct FakeWire
{
  uint8_t address = 0;
  bool begin(int, int) { return true; }
  bool setClock(uint32_t) { return true; }
  void setTimeOut(int) {}
  void beginTransmission(uint8_t value) { address = value; }
  uint8_t endTransmission() const
  {
    return Fake::boardsPresent && (address == 0x40 || address == 0x41) ? 0 : 2;
  }
};
inline FakeWire Wire;

struct FakeWiFi
{
  bool mode(int value)
  {
    Fake::radioOutputEnable.push_back(Fake::outputEnable);
    unsigned active = 0;
    for (auto& board : Fake::pwm)
      for (uint16_t ticks : board)
        if (ticks != 4096)
          ++active;
    Fake::radioActiveChannels.push_back(active);
    Fake::wifiMode = value;
    return !Fake::radioFailure;
  }
  std::string macAddress() const { return "00:11:22:33:44:55"; }
  bool softAP(const char* name, const char* password, int channel, int, int)
  {
    if (Fake::apFailure || password != nullptr)
      return false;
    Fake::apStarted = true;
    Fake::apName = name;
    Fake::channel = uint8_t(channel);
    return true;
  }
  bool softAPdisconnect(bool)
  {
    if (Fake::apStopFailure)
      return false;
    Fake::apStarted = false;
    Fake::wifiMode = WIFI_STA;
    return true;
  }
  struct Address { std::string toString() const { return "192.168.4.1"; } };
  Address softAPIP() const { return {}; }
};
inline FakeWiFi WiFi;
inline int esp_wifi_set_ps(int) { return ESP_OK; }
inline int esp_wifi_set_channel(uint8_t channel, int) { Fake::channel = channel; return ESP_OK; }
struct esp_now_recv_info_t {};
inline int esp_now_init() { return Fake::radioFailure ? -1 : ESP_OK; }
inline int esp_now_register_recv_cb(void (*)(const esp_now_recv_info_t*, const uint8_t*, int)) { return ESP_OK; }

struct FakeQueue
{
  std::vector<uint8_t> data;
  bool available = false;
};
using QueueHandle_t = FakeQueue*;
inline QueueHandle_t xQueueCreate(int, size_t size)
{
  auto* queue = new FakeQueue;
  queue->data.resize(size);
  return queue;
}
inline int xQueueReset(QueueHandle_t handle) { handle->available = false; return pdPASS; }
inline int xQueueOverwrite(QueueHandle_t handle, const void* packet)
{
  memcpy(handle->data.data(), packet, handle->data.size());
  handle->available = true;
  return pdPASS;
}
inline int xQueueSend(QueueHandle_t handle, const void* packet, int)
{
  return handle->available ? pdFALSE : xQueueOverwrite(handle, packet);
}
inline int xQueueReceive(QueueHandle_t handle, void* packet, int ticks)
{
  if (!handle->available && ticks > 0)
  {
    if (Fake::waitHook != nullptr)
      Fake::waitHook();
    if (!handle->available)
      Fake::now += uint32_t(ticks);
  }
  if (!handle->available)
    return pdFALSE;
  memcpy(packet, handle->data.data(), handle->data.size());
  handle->available = false;
  return pdTRUE;
}

class Adafruit_PWMServoDriver
{
  size_t board;
  uint32_t oscillator = 25000000;
  uint8_t prescale = 121;
public:
  explicit Adafruit_PWMServoDriver(uint8_t address) : board(address - 0x40) {}
  bool begin()
  {
    Fake::driverOutputEnable.push_back(Fake::outputEnable);
    return Fake::boardsPresent;
  }
  void setOscillatorFrequency(uint32_t value) { oscillator = value; }
  void setPWMFreq(float hz) { prescale = uint8_t(lroundf(float(oscillator) / (4096 * hz)) - 1); }
  uint8_t readPrescale() const { return prescale; }
  int setPWM(uint8_t channel, uint16_t on, uint16_t off)
  {
    if (Fake::writeFailure || Fake::writesUntilFailure == 0)
      return 1;
    if (Fake::writesUntilFailure > 0)
      --Fake::writesUntilFailure;
    Fake::pwmOn[board][channel] = on;
    Fake::pwmOff[board][channel] = off;
    Fake::pwm[board][channel] = off == 4096 ? 4096 : uint16_t((off + 4096 - on) % 4096);
    Fake::pwmWrites.push_back({Fake::now, board, channel, on, off, Fake::outputEnable});
    return 0;
  }
};

class Preferences
{
public:
  bool begin(const char*, bool) { return !Fake::storageFailure; }
  bool isKey(const char*) { return !Fake::saved.empty(); }
  size_t getBytesLength(const char*) { return Fake::saved.size(); }
  size_t getBytes(const char*, void* target, size_t size)
  {
    if (Fake::readFailure || size != Fake::saved.size())
      return 0;
    memcpy(target, Fake::saved.data(), size);
    return size;
  }
  size_t putBytes(const char*, const void* source, size_t size)
  {
    if (Fake::storageFailure)
      return 0;
    const auto* bytes = static_cast<const uint8_t*>(source);
    Fake::saved.assign(bytes, bytes + size);
    ++Fake::saves;
    return size;
  }
};
