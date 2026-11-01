#pragma once

#include "HexapodControl.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <type_traits>

namespace Hexapod
{
struct FloatParameter
{
  const char* name;
  float Settings::*member;
  float minimum;
  float maximum;
};

struct IntegerParameter
{
  const char* name;
  uint32_t Settings::*member;
  uint32_t minimum;
  uint32_t maximum;
};

constexpr FloatParameter FLOAT_PARAMETERS[] =
{
  {"coxa_length", &Settings::coxaLength, 1, 300},
  {"femur_length", &Settings::femurLength, 1, 300},
  {"tibia_length", &Settings::tibiaLength, 1, 400},
  {"step_length", &Settings::stepLength, 0, 150},
  {"step_height", &Settings::stepHeight, 0, 150},
  {"home_z", &Settings::homeZ, -200, -1},
  {"smoothing", &Settings::smoothing, 0.01f, 1},
  {"rotation_step", &Settings::rotationStep, 0, 45},
  {"phase_step", &Settings::phaseStep, 0.001f, 0.05f},
  {"coxa_min", &Settings::coxaMin, 0, 180},
  {"coxa_max", &Settings::coxaMax, 0, 180},
  {"servo_hz", &Settings::servoHz, 40, 60},
  {"motion_stride", &Settings::motionStride, 0, 80},
  {"motion_lift", &Settings::motionLift, 0, 80},
  {"motion_turn", &Settings::motionTurn, 0, 20}
};

constexpr IntegerParameter INTEGER_PARAMETERS[] =
{
  {"update_ms", &Settings::updateMs, 5, 50},
  {"oscillator0_hz", &Settings::oscillator0Hz, 20000000, 30000000},
  {"oscillator1_hz", &Settings::oscillator1Hz, 20000000, 30000000},
  {"i2c_hz", &Settings::i2cHz, 100000, 400000},
  {"radio_channel", &Settings::radioChannel, 1, 13},
  {"timeout_ms", &Settings::timeoutMs, 100, 2000},
  {"stick_center", &Settings::stickCenter, 0, 4095},
  {"stick_deadzone", &Settings::stickDeadzone, 1, 1024},
  {"throttle_channel", &Settings::throttleChannel, 0, 5},
  {"steering_channel", &Settings::steeringChannel, 0, 5},
  {"throttle_reverse", &Settings::throttleReversed, 0, 1},
  {"steering_reverse", &Settings::steeringReversed, 0, 1},
  {"motion_cycle_ms", &Settings::motionCycleMs, 600, 6000},
  {"strafe_channel", &Settings::strafeChannel, 0, 5},
  {"strafe_reverse", &Settings::strafeReversed, 0, 1}
};

inline bool validateSettings(const Settings& settings, const char*& error)
{
  for (const FloatParameter& parameter : FLOAT_PARAMETERS)
  {
    const float value = settings.*(parameter.member);
    if (!isfinite(value) || value < parameter.minimum || value > parameter.maximum)
    {
      error = parameter.name;
      return false;
    }
  }
  for (const IntegerParameter& parameter : INTEGER_PARAMETERS)
    if (settings.*(parameter.member) < parameter.minimum ||
        settings.*(parameter.member) > parameter.maximum)
    {
      error = parameter.name;
      return false;
    }
  if (settings.gait >= GAIT_COUNT)
    error = "unknown gait";
  else if (settings.gait != uint32_t(Gait::Legacy) &&
           (settings.strafeChannel == settings.throttleChannel ||
            settings.strafeChannel == settings.steeringChannel))
    error = "strafe, throttle and steering channels must differ for improved gaits";
  else if (settings.coxaMin >= settings.coxaMax)
    error = "coxa_min must be less than coxa_max";
  else if (settings.stickCenter < settings.stickDeadzone ||
           settings.stickCenter + settings.stickDeadzone > 4095)
    error = "stick center +/- deadzone must fit 0..4095";
  else if (settings.throttleChannel == settings.steeringChannel)
    error = "throttle and steering channels must differ";
  else
  {
    const float reach = (settings.femurLength + settings.tibiaLength) * 0.70710678f;
    const float distance = hypotf(reach, settings.homeZ);
    if (distance <= fabsf(settings.femurLength - settings.tibiaLength) + 0.001f ||
        distance >= settings.femurLength + settings.tibiaLength - 0.001f)
      error = "home stance is unreachable with these link lengths/home_z";
    else
      error = nullptr;
  }
  if (error != nullptr)
    return false;
  for (size_t leg = 0; leg < LEG_COUNT; ++leg)
  {
    const LegOrigin& origin = settings.origins[leg];
    if (!isfinite(origin.x) || !isfinite(origin.y) ||
        fabsf(origin.x) > 500 || fabsf(origin.y) > 500 ||
        hypotf(origin.x, origin.y) < 1)
    {
      error = "leg origin must be within +/-500 mm and at least 1 mm from center";
      return false;
    }
    for (const ServoCalibration& servo : settings.servos[leg])
    {
      uint16_t pulse = 0;
      if (!servoPulse(90, servo, pulse) || fabsf(servo.trimDegrees) > 90)
      {
        error = "calibration requires 500 <= min < max <= 2500, trim +/-90, reverse 0/1";
        return false;
      }
    }
  }
  return true;
}

inline bool parseNumber(const char* text, float minimum, float maximum, float& result)
{
  char* end = nullptr;
  errno = 0;
  const float value = strtof(text, &end);
  if (end == text || *end != '\0' || errno == ERANGE ||
      !isfinite(value) || value < minimum || value > maximum)
    return false;
  result = value;
  return true;
}

inline bool parseInteger(const char* text, uint32_t minimum, uint32_t maximum, uint32_t& result)
{
  if (*text == '\0')
    return false;
  uint32_t value = 0;
  for (; *text; ++text)
  {
    if (*text < '0' || *text > '9')
      return false;
    const uint32_t digit = uint32_t(*text - '0');
    if (digit > maximum || value > (maximum - digit) / 10)
      return false;
    value = value * 10 + digit;
  }
  if (value < minimum)
    return false;
  result = value;
  return true;
}

inline bool setParameter(Settings& settings, const char* name, const char* text, const char*& error)
{
  Settings candidate = settings;
  bool found = false;
  for (const FloatParameter& parameter : FLOAT_PARAMETERS)
    if (strcmp(name, parameter.name) == 0)
    {
      found = parseNumber(text, parameter.minimum, parameter.maximum,
                          candidate.*(parameter.member));
      if (!found)
      {
        error = "invalid number/range; see params";
        return false;
      }
      break;
    }
  if (!found)
    for (const IntegerParameter& parameter : INTEGER_PARAMETERS)
      if (strcmp(name, parameter.name) == 0)
      {
        found = parseInteger(text, parameter.minimum, parameter.maximum,
                             candidate.*(parameter.member));
        if (!found)
        {
          error = "invalid integer/range; see params";
          return false;
        }
        break;
      }
  if (!found)
  {
    error = "unknown parameter; see params";
    return false;
  }
  if (!validateSettings(candidate, error))
    return false;
  settings = candidate;
  return true;
}

enum class CommandType
{
  Help, Status, Show, Params, Save, Load, Defaults, Init, Scan, Off, Stop, Home, Radio, Web,
  Center, Servo, Pulse, Joints, Foot, Walk, TestServos, Next, Retest, Set, Calibrate, Origin,
  Gait, Tune
};

struct Command
{
  CommandType type = CommandType::Help;
  size_t leg = LEG_COUNT;
  size_t joint = 0;
  float values[4] = {};
  uint32_t durationMs = 0;
  Gait gait = Gait::Legacy;
  const char* name = nullptr;
  const char* text = nullptr;
};

constexpr const char* LEG_NAMES[] = {"rf", "rm", "rr", "lr", "lm", "lf"};
constexpr const char* JOINT_NAMES[] = {"coxa", "femur", "tibia"};

inline bool parseIndex(const char* text, const char* const* names, size_t count, size_t& index)
{
  for (size_t i = 0; i < count; ++i)
    if (strcmp(text, names[i]) == 0)
    {
      index = i;
      return true;
    }
  uint32_t value = 0;
  if (!parseInteger(text, 0, uint32_t(count - 1), value))
    return false;
  index = value;
  return true;
}

inline bool parseCommand(char* line, Command& command, const char*& error)
{
  char* tokens[8] = {};
  size_t count = 0;
  for (char* cursor = line; *cursor;)
  {
    if (*cursor == ' ' || *cursor == '\t')
    {
      ++cursor;
      continue;
    }
    if (count == 8)
    {
      error = "too many arguments";
      return false;
    }
    tokens[count++] = cursor;
    while (*cursor && *cursor != ' ' && *cursor != '\t')
    {
      if (*cursor >= 'A' && *cursor <= 'Z')
        *cursor = char(*cursor + ('a' - 'A'));
      ++cursor;
    }
    if (*cursor)
      *cursor++ = '\0';
  }
  error = "invalid command/arguments; use help";
  if (count == 0)
    return false;
  Command parsed;
  const struct { const char* name; CommandType type; } simple[] =
  {
    {"help", CommandType::Help}, {"status", CommandType::Status},
    {"show", CommandType::Show}, {"params", CommandType::Params},
    {"save", CommandType::Save}, {"load", CommandType::Load},
    {"defaults", CommandType::Defaults}, {"init", CommandType::Init},
    {"scan", CommandType::Scan}, {"off", CommandType::Off}, {"stop", CommandType::Stop},
    {"home", CommandType::Home}, {"radio", CommandType::Radio}, {"web", CommandType::Web},
    {"testservos", CommandType::TestServos}, {"next", CommandType::Next},
    {"retest", CommandType::Retest}
  };
  for (const auto& item : simple)
    if (strcmp(tokens[0], item.name) == 0 && count == 1)
    {
      parsed.type = item.type;
      command = parsed;
      return true;
    }

  if (strcmp(tokens[0], "gait") == 0 && count == 2)
  {
    size_t gait = 0;
    if (!parseIndex(tokens[1], GAIT_NAMES, GAIT_COUNT, gait))
      return false;
    parsed.type = CommandType::Gait;
    parsed.gait = Gait(gait);
  }
  else if ((strcmp(tokens[0], "set") == 0 || strcmp(tokens[0], "tune") == 0) && count == 3)
  {
    parsed.type = strcmp(tokens[0], "tune") == 0 ? CommandType::Tune : CommandType::Set;
    parsed.name = tokens[1];
    parsed.text = tokens[2];
  }
  else if (strcmp(tokens[0], "walk") == 0 && count == 4)
  {
    parsed.type = CommandType::Walk;
    if (!parseNumber(tokens[1], -1, 1, parsed.values[0]) ||
        !parseNumber(tokens[2], -1, 1, parsed.values[1]) ||
        !parseInteger(tokens[3], 100, 10000, parsed.durationMs))
      return false;
  }
  else if (strcmp(tokens[0], "move") == 0 && count == 5)
  {
    parsed.type = CommandType::Walk;
    if (!parseNumber(tokens[1], -1, 1, parsed.values[0]) ||
        !parseNumber(tokens[2], -1, 1, parsed.values[2]) ||
        !parseNumber(tokens[3], -1, 1, parsed.values[1]) ||
        !parseInteger(tokens[4], 100, 10000, parsed.durationMs))
      return false;
  }
  else if (strcmp(tokens[0], "center") == 0 && (count == 1 || count == 2))
  {
    parsed.type = CommandType::Center;
    if (count == 2 && strcmp(tokens[1], "all") != 0 &&
        !parseIndex(tokens[1], LEG_NAMES, LEG_COUNT, parsed.leg))
      return false;
  }
  else
  {
    if (count < 3 || !parseIndex(tokens[1], LEG_NAMES, LEG_COUNT, parsed.leg))
      return false;
    if (strcmp(tokens[0], "origin") == 0 && count == 4)
    {
      parsed.type = CommandType::Origin;
      if (!parseNumber(tokens[2], -500, 500, parsed.values[0]) ||
          !parseNumber(tokens[3], -500, 500, parsed.values[1]))
        return false;
    }
    else if ((strcmp(tokens[0], "joints") == 0 || strcmp(tokens[0], "foot") == 0) && count == 5)
    {
      const bool foot = strcmp(tokens[0], "foot") == 0;
      parsed.type = foot ? CommandType::Foot : CommandType::Joints;
      for (size_t i = 0; i < 3; ++i)
        if (!parseNumber(tokens[i + 2], foot ? -150.0f : 0.0f, foot ? 150.0f : 180.0f,
                         parsed.values[i]))
          return false;
    }
    else
    {
      if (!parseIndex(tokens[2], JOINT_NAMES, JOINT_COUNT, parsed.joint))
        return false;
      if (strcmp(tokens[0], "servo") == 0 && count == 4)
      {
        parsed.type = CommandType::Servo;
        if (!parseNumber(tokens[3], 0, 180, parsed.values[0]))
          return false;
      }
      else if (strcmp(tokens[0], "pulse") == 0 && count == 4)
      {
        parsed.type = CommandType::Pulse;
        uint32_t pulse = 0;
        if (!parseInteger(tokens[3], 500, 2500, pulse))
          return false;
        parsed.values[0] = float(pulse);
      }
      else if (strcmp(tokens[0], "cal") == 0 && count == 7)
      {
        parsed.type = CommandType::Calibrate;
        uint32_t minimum = 0, maximum = 0, reverse = 0;
        if (!parseInteger(tokens[3], 500, 2500, minimum) ||
            !parseInteger(tokens[4], 500, 2500, maximum) || minimum >= maximum ||
            !parseNumber(tokens[5], -90, 90, parsed.values[2]) ||
            !parseInteger(tokens[6], 0, 1, reverse))
          return false;
        parsed.values[0] = float(minimum);
        parsed.values[1] = float(maximum);
        parsed.values[3] = float(reverse);
      }
      else
        return false;
    }
  }
  command = parsed;
  return true;
}

enum class LineEvent { None, Ready, Empty, Overflow, Invalid, Disable };

struct ConsoleLine
{
  char data[160] = {};
  size_t length = 0;
  bool discarding = false;
  bool skipNextLf = false;

  LineEvent push(char character)
  {
    if (character == '\n' && skipNextLf)
    {
      skipNextLf = false;
      return LineEvent::None;
    }
    skipNextLf = character == '\r';
    if (character == '!')
    {
      length = 0;
      discarding = true;
      return LineEvent::Disable;
    }
    if (character == '\r' || character == '\n')
    {
      data[length] = '\0';
      const LineEvent event = discarding ? LineEvent::None :
          (length != 0 ? LineEvent::Ready : LineEvent::Empty);
      length = 0;
      discarding = false;
      return event;
    }
    if (discarding)
      return LineEvent::None;
    if (character == '\b' || character == 127)
    {
      if (length != 0)
        --length;
      return LineEvent::None;
    }
    if ((character < 32 && character != '\t') || character > 126)
    {
      discarding = true;
      length = 0;
      return LineEvent::Invalid;
    }
    if (length + 1 == sizeof(data))
    {
      discarding = true;
      length = 0;
      return LineEvent::Overflow;
    }
    data[length++] = character;
    return LineEvent::None;
  }
};

struct TimedMotion
{
  float direction = 0;
  float rotation = 0;
  uint32_t startedMs = 0;
  uint32_t durationMs = 0;
  float strafe = 0;

  bool expired(uint32_t now) const
  {
    return uint32_t(now - startedMs) >= durationMs;
  }
};

struct SettingsRecord
{
  uint32_t magic = 0x48585044;
  uint32_t version = 2;
  Settings settings;
  uint32_t checksum = 0;
};

static_assert(std::is_trivially_copyable<SettingsRecord>::value, "Settings must remain plain data");

constexpr size_t SETTINGS_V1_BYTES = 360;
struct SettingsRecordV1
{
  uint32_t magic;
  uint32_t version;
  uint8_t settings[SETTINGS_V1_BYTES];
  uint32_t checksum;
};
static_assert(std::is_standard_layout<Settings>::value &&
              offsetof(Settings, gait) == SETTINGS_V1_BYTES,
              "Do not change the version-1 settings prefix");
static_assert(sizeof(SettingsRecordV1) == 372, "Version-1 record layout changed");

inline uint32_t settingsBytesChecksum(const void* data, size_t size)
{
  const auto* bytes = static_cast<const uint8_t*>(data);
  uint32_t checksum = 2166136261u;
  for (size_t i = 0; i < size; ++i)
    checksum = (checksum ^ bytes[i]) * 16777619u;
  return checksum;
}

inline uint32_t settingsChecksum(const Settings& settings)
{
  return settingsBytesChecksum(&settings, sizeof(settings));
}

inline SettingsRecord makeSettingsRecord(const Settings& settings)
{
  SettingsRecord record;
  memcpy(&record.settings, &settings, sizeof(settings));
  record.checksum = settingsChecksum(record.settings);
  return record;
}

inline bool decodeSettings(const void* bytes, size_t length, Settings& settings, const char*& error,
                           bool* migrated = nullptr)
{
  if (migrated != nullptr)
    *migrated = false;
  if (bytes == nullptr || (length != sizeof(SettingsRecord) && length != sizeof(SettingsRecordV1)))
  {
    error = "saved settings have an incompatible size";
    return false;
  }
  Settings candidate;
  const bool legacy = length == sizeof(SettingsRecordV1);
  if (legacy)
  {
    SettingsRecordV1 record;
    memcpy(&record, bytes, sizeof(record));
    if (record.magic != 0x48585044 || record.version != 1 ||
        record.checksum != settingsBytesChecksum(record.settings, sizeof(record.settings)))
    {
      error = "saved settings version/checksum mismatch";
      return false;
    }
    memcpy(&candidate, record.settings, SETTINGS_V1_BYTES);
  }
  else
  {
    SettingsRecord record;
    memcpy(&record, bytes, sizeof(record));
    if (record.magic != 0x48585044 || record.version != 2 ||
        record.checksum != settingsChecksum(record.settings))
    {
      error = "saved settings version/checksum mismatch";
      return false;
    }
    candidate = record.settings;
  }
  if (!validateSettings(candidate, error))
    return false;
  settings = candidate;
  if (migrated != nullptr)
    *migrated = legacy;
  return true;
}
}
