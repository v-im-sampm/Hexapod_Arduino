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

inline float stickCommand(uint16_t value)
{
  if (value < STICK_CENTER - STICK_DEADZONE)
    return 1.0f;
  if (value > STICK_CENTER + STICK_DEADZONE)
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

  bool expire(uint32_t now)
  {
    if (!hasSignal || uint32_t(now - lastPacketMs) < CONTROL_TIMEOUT_MS)
      return false;
    direction = rotation = 0.0f;
    armed = hasSignal = false;
    return true;
  }

  bool receive(const ControlPacket& packet, uint32_t now)
  {
    expire(now);
    if (uint32_t(now - packet.receivedAtMs) >= CONTROL_TIMEOUT_MS)
      return false;

    lastPacketMs = packet.receivedAtMs;
    hasSignal = true;
    const float nextDirection = stickCommand(packet.channels[2]);
    const float nextRotation = stickCommand(packet.channels[4]);
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

struct ServoCalibration
{
  uint16_t minUs = 1000;
  uint16_t maxUs = 2000;
  float trimDegrees = 0.0f;
  bool reversed = false;
};

inline size_t servoBoard(size_t legIndex)
{
  return legIndex / 3;
}

inline uint8_t servoChannel(size_t legIndex, size_t jointIndex)
{
  return uint8_t((legIndex % 3) * JOINT_COUNT + jointIndex);
}

inline bool servoPulse(float angle, const ServoCalibration& calibration, uint16_t& pulseUs)
{
  if (!isfinite(angle) || !isfinite(calibration.trimDegrees) ||
      calibration.minUs < 500 || calibration.maxUs > 2500 ||
      calibration.minUs >= calibration.maxUs)
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
