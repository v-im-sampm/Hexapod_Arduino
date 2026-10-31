#pragma once

#include <stddef.h>
#include <stdint.h>
#include <math.h>

namespace Hexapod
{
constexpr size_t LEG_COUNT = 6;
constexpr size_t JOINT_COUNT = 3;
constexpr size_t DRIVER_COUNT = 2;
constexpr size_t CONTROL_CHANNEL_COUNT = 10;
constexpr size_t MAX_PACKET_LENGTH = 37;
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t CONTROL_TIMEOUT_MS = 500;
constexpr uint16_t STICK_CENTER = 2047;
constexpr uint16_t STICK_DEADZONE = 100;

struct ServoCalibration
{
  uint16_t minUs = 1000;
  uint16_t maxUs = 2000;
  float trimDegrees = 0.0f;
  uint8_t reversed = 0;
};

struct LegOrigin
{
  float x;
  float y;
};

struct Settings
{
  float coxaLength = 38.0f;
  float femurLength = 86.25f;
  float tibiaLength = 160.5f;
  float stepLength = 60.0f;
  float stepHeight = 100.0f;
  float homeZ = -40.0f;
  float smoothing = 0.4f;
  float rotationStep = 15.0f;
  float phaseStep = 0.015f;
  float coxaMin = 45.0f;
  float coxaMax = 135.0f;
  float servoHz = 50.0f;
  uint32_t updateMs = 10;
  uint32_t oscillator0Hz = 25000000;
  uint32_t oscillator1Hz = 25000000;
  uint32_t i2cHz = 400000;
  uint32_t radioChannel = ESPNOW_CHANNEL;
  uint32_t timeoutMs = CONTROL_TIMEOUT_MS;
  uint32_t stickCenter = STICK_CENTER;
  uint32_t stickDeadzone = STICK_DEADZONE;
  uint32_t throttleChannel = 2;
  uint32_t steeringChannel = 4;
  uint32_t throttleReversed = 0;
  uint32_t steeringReversed = 0;
  LegOrigin origins[LEG_COUNT] =
  {
    {100.94f, -57.11f}, {0.0f, -105.72f}, {-100.94f, -57.11f},
    {-100.94f, 57.11f}, {0.0f, 105.72f}, {100.94f, 57.11f}
  };
  ServoCalibration servos[LEG_COUNT][JOINT_COUNT] = {};

  uint32_t oscillatorHz(size_t board) const
  {
    return board == 0 ? oscillator0Hz : oscillator1Hz;
  }
};

struct ControlPacket
{
  uint16_t channels[CONTROL_CHANNEL_COUNT] = {};
  uint32_t receivedAtMs = 0;
};

inline bool parseControlPacket(const uint8_t* data, size_t length, ControlPacket& result)
{
  if (data == nullptr || length == 0 || length > MAX_PACKET_LENGTH)
    return false;

  ControlPacket packet;
  size_t position = 0;
  for (size_t channel = 0; channel < CONTROL_CHANNEL_COUNT; ++channel)
  {
    uint16_t value = 0;
    size_t digits = 0;
    const size_t maxDigits = channel < 6 ? 4 : 1;
    const uint16_t maximum = channel < 6 ? 4095 : 1;

    while (position < length && data[position] != ',')
    {
      const uint8_t digit = data[position++];
      if (digit < '0' || digit > '9' || ++digits > maxDigits)
        return false;
      value = value * 10 + (digit - '0');
      if (value > maximum)
        return false;
    }

    if (digits == 0)
      return false;
    packet.channels[channel] = value;

    if (channel + 1 < CONTROL_CHANNEL_COUNT)
    {
      if (position == length)
        return false;
      ++position;
    }
    else if (position != length)
      return false;
  }

  result = packet;
  return true;
}

inline float stickCommand(uint16_t value, uint32_t center = STICK_CENTER,
                          uint32_t deadzone = STICK_DEADZONE)
{
  if (value < center - deadzone)
    return 1.0f;
  if (value > center + deadzone)
    return -1.0f;
  return 0.0f;
}

struct ControlState
{
  float direction = 0.0f;
  float rotation = 0.0f;
  bool armed = false;
  bool hasSignal = false;
  uint32_t lastPacketMs = 0;

  bool expire(uint32_t now, uint32_t timeoutMs = CONTROL_TIMEOUT_MS)
  {
    if (!hasSignal || uint32_t(now - lastPacketMs) < timeoutMs)
      return false;
    direction = rotation = 0.0f;
    armed = hasSignal = false;
    return true;
  }

  bool receive(const ControlPacket& packet, uint32_t now, const Settings& settings = Settings{})
  {
    expire(now, settings.timeoutMs);
    if (uint32_t(now - packet.receivedAtMs) >= settings.timeoutMs)
      return false;

    lastPacketMs = packet.receivedAtMs;
    hasSignal = true;
    const float nextDirection = stickCommand(packet.channels[settings.throttleChannel],
        settings.stickCenter, settings.stickDeadzone) * (settings.throttleReversed ? -1 : 1);
    const float nextRotation = stickCommand(packet.channels[settings.steeringChannel],
        settings.stickCenter, settings.stickDeadzone) * (settings.steeringReversed ? -1 : 1);
    if (!armed && nextDirection == 0.0f && nextRotation == 0.0f)
      armed = true;
    if (armed)
    {
      direction = nextDirection;
      rotation = nextRotation;
    }
    return true;
  }
};

inline size_t servoBoard(size_t legIndex)
{
  return legIndex / 3;
}

inline uint8_t servoChannel(size_t legIndex, size_t jointIndex)
{
  return uint8_t((legIndex % 3) * JOINT_COUNT + jointIndex + (legIndex < 3 ? 7 : 0));
}

inline bool servoPulse(float angle, const ServoCalibration& calibration, uint16_t& pulseUs)
{
  if (!isfinite(angle) || !isfinite(calibration.trimDegrees) ||
      calibration.minUs < 500 || calibration.maxUs > 2500 ||
      calibration.minUs >= calibration.maxUs || calibration.reversed > 1)
    return false;

  if (calibration.reversed)
    angle = 180.0f - angle;
  angle = fminf(180.0f, fmaxf(0.0f, angle + calibration.trimDegrees));
  pulseUs = uint16_t(lroundf(calibration.minUs +
      angle * (calibration.maxUs - calibration.minUs) / 180.0f));
  return true;
}

inline uint16_t pulseToTicks(uint16_t pulseUs, uint32_t oscillatorHz, uint8_t prescale)
{
  const uint64_t denominator = uint64_t(prescale + 1) * 1000000;
  return uint16_t((uint64_t(pulseUs) * oscillatorHz + denominator / 2) / denominator);
}
}
