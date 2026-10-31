#include "../HexapodConsole.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <limits>

using namespace Hexapod;

void check(bool condition, const char* expression, int line)
{
  if (!condition)
  {
    fprintf(stderr, "FAIL line %d: %s\n", line, expression);
    exit(1);
  }
}

#define CHECK(condition) check((condition), #condition, __LINE__)

bool parse(const std::string& message, ControlPacket& packet)
{
  return parseControlPacket(reinterpret_cast<const uint8_t*>(message.data()),
                            message.size(), packet);
}

std::string telegram(const uint16_t channels[CONTROL_CHANNEL_COUNT])
{
  std::string result;
  for (size_t i = 0; i < CONTROL_CHANNEL_COUNT; ++i)
  {
    if (i != 0)
      result += ',';
    result += std::to_string(channels[i]);
  }
  return result;
}

ControlPacket packetAt(uint32_t time, uint16_t throttle = 2047, uint16_t steering = 2047)
{
  ControlPacket packet;
  CHECK(parse("0,2047,2047,2047,2047,2047,1,1,1,1", packet));
  packet.channels[2] = throttle;
  packet.channels[4] = steering;
  packet.receivedAtMs = time;
  return packet;
}

void testProtocol()
{
  ControlPacket packet;
  const std::string maximum = "4095,4095,4095,4095,4095,4095,1,1,1,1";
  CHECK(maximum.size() == MAX_PACKET_LENGTH);
  CHECK(parse(maximum, packet));
  CHECK(packet.channels[5] == 4095 && packet.channels[9] == 1);
  CHECK(parse("0,0,0,0,0,0,0,0,0,0", packet));

  const std::string invalid[] =
  {
    "", "0", "0,0,0,0,0,0,1,1,1", "0,0,0,0,0,0,1,1,1,1,1",
    ",0,0,0,0,0,1,1,1,1", "0,,0,0,0,0,1,1,1,1",
    "0,0,0,0,0,0,1,1,1,", "0,0,0,0,0,0,1,1,1,1,",
    "-1,0,0,0,0,0,1,1,1,1", "+1,0,0,0,0,0,1,1,1,1",
    " 0,0,0,0,0,0,1,1,1,1", "0,0,0,0,0,0,1,1,1,1\n",
    "0.0,0,0,0,0,0,1,1,1,1", "x,0,0,0,0,0,1,1,1,1",
    "65536,0,0,0,0,0,1,1,1,1", "00000,0,0,0,0,0,1,1,1,1",
    "0,0,0,0,0,0,01,1,1,1", maximum + std::string(1, '\0'),
    std::string(250, '9'), std::string("0,0,0,0,0,0,1,1,1,") + '\0'
  };
  for (const std::string& message : invalid)
  {
    packet = packetAt(123);
    CHECK(!parse(message, packet));
    CHECK(packet.receivedAtMs == 123 && packet.channels[2] == 2047);
  }
  CHECK(!parseControlPacket(nullptr, 19, packet));

  // Match tx.py's decimal formatting for every ADC value in every field.
  uint16_t channels[CONTROL_CHANNEL_COUNT] = {0, 2047, 2047, 2047, 2047, 2047, 1, 1, 1, 1};
  for (size_t channel = 0; channel < CONTROL_CHANNEL_COUNT; ++channel)
  {
    const uint16_t maximumValue = channel < 6 ? 4095 : 1;
    const uint16_t saved = channels[channel];
    for (uint16_t value = 0; value <= maximumValue; ++value)
    {
      channels[channel] = value;
      CHECK(parse(telegram(channels), packet));
      for (size_t i = 0; i < CONTROL_CHANNEL_COUNT; ++i)
        CHECK(packet.channels[i] == channels[i]);
    }
    channels[channel] = maximumValue + 1;
    CHECK(!parse(telegram(channels), packet));
    channels[channel] = saved;
  }
}

void testControl()
{
  CHECK(ESPNOW_CHANNEL == 1 && CONTROL_TIMEOUT_MS == 500);
  for (uint16_t value = 0; value <= 4095; ++value)
    CHECK(stickCommand(value) == (value < 1947 ? 1.0f : value > 2147 ? -1.0f : 0.0f));

  ControlState state;
  CHECK(!state.armed && state.direction == 0 && state.rotation == 0);
  CHECK(!state.expire(10000));
  CHECK(state.receive(packetAt(0, 0, 4095), 0));
  CHECK(!state.armed && state.direction == 0 && state.rotation == 0);
  CHECK(state.receive(packetAt(20, 1947, 2147), 20));
  CHECK(state.armed);
  CHECK(state.receive(packetAt(40, 0, 4095), 40));
  CHECK(state.direction == 1 && state.rotation == -1);
  CHECK(!state.expire(539));
  CHECK(state.expire(540));
  CHECK(!state.armed && !state.hasSignal && state.direction == 0 && state.rotation == 0);
  CHECK(!state.expire(541));
  CHECK(state.receive(packetAt(550, 4095, 0), 550));
  CHECK(!state.armed && state.direction == 0 && state.rotation == 0);
  CHECK(state.receive(packetAt(570), 570));
  CHECK(state.armed);
  CHECK(state.receive(packetAt(590, 4095, 0), 590));
  CHECK(state.direction == -1 && state.rotation == 1);

  // A fresh moving packet after a gap cannot bypass neutral-stick re-arming.
  CHECK(state.receive(packetAt(1090, 0, 0), 1090));
  CHECK(!state.armed && state.direction == 0 && state.rotation == 0);
  CHECK(!state.receive(packetAt(1100), 1600));
  CHECK(!state.armed && !state.hasSignal);
  CHECK(state.receive(packetAt(1700), 1900));
  CHECK(!state.expire(2199));
  CHECK(state.expire(2200)); // Timeout starts at reception, not processing.

  state = ControlState{};
  const uint32_t nearWrap = UINT32_MAX - 100;
  CHECK(state.receive(packetAt(nearWrap), nearWrap));
  CHECK(state.receive(packetAt(nearWrap + 20, 0, 0), nearWrap + 20));
  CHECK(!state.expire(uint32_t(nearWrap + 519)));
  CHECK(state.expire(uint32_t(nearWrap + 520)));

  state = ControlState{};
  CHECK(state.receive(packetAt(0), 0));
  CHECK(state.receive(packetAt(20, 0, 0), 20));
  ControlPacket invalid;
  CHECK(!parse("0,0,0,0,0,0,1,1,1,2", invalid));
  CHECK(state.expire(520)); // Malformed traffic does not refresh the watchdog.
}

void testServos()
{
  bool used[DRIVER_COUNT][16] = {};
  for (size_t leg = 0; leg < LEG_COUNT; ++leg)
  {
    CHECK(servoBoard(leg) == (leg < 3 ? 0 : 1));
    for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
    {
      const uint8_t channel = servoChannel(leg, joint);
      CHECK(channel == (leg % 3) * 3 + joint);
      CHECK(!used[servoBoard(leg)][channel]);
      used[servoBoard(leg)][channel] = true;
    }
  }
  for (size_t board = 0; board < DRIVER_COUNT; ++board)
    for (size_t channel = 0; channel < 16; ++channel)
      CHECK(used[board][channel] == (channel < 9));

  ServoCalibration calibration;
  uint16_t pulse = 0;
  CHECK(servoPulse(0, calibration, pulse) && pulse == 1000);
  CHECK(servoPulse(90, calibration, pulse) && pulse == 1500);
  CHECK(servoPulse(180, calibration, pulse) && pulse == 2000);
  CHECK(servoPulse(-100, calibration, pulse) && pulse == 1000);
  CHECK(servoPulse(300, calibration, pulse) && pulse == 2000);
  CHECK(pulseToTicks(1000, 25000000, 121) == 205);
  CHECK(pulseToTicks(1500, 25000000, 121) == 307);
  CHECK(pulseToTicks(2000, 25000000, 121) == 410);
  const double frequency = 25000000.0 / (4096 * 122);
  CHECK(frequency > 49.9 && frequency < 50.1);
  for (int angle = 0; angle <= 180; ++angle)
  {
    CHECK(servoPulse(float(angle), calibration, pulse));
    const uint16_t ticks = pulseToTicks(pulse, 25000000, 121);
    CHECK(ticks >= 205 && ticks <= 410);
    CHECK(fabs(ticks * 4.88 - pulse) <= 2.44 + 0.0001);
  }
  calibration.reversed = true;
  CHECK(servoPulse(0, calibration, pulse) && pulse == 2000);
  CHECK(servoPulse(180, calibration, pulse) && pulse == 1000);
  calibration.trimDegrees = 9;
  CHECK(servoPulse(90, calibration, pulse) && pulse == 1550);
  calibration.trimDegrees = 200;
  CHECK(servoPulse(90, calibration, pulse) && pulse == 2000);
  calibration = ServoCalibration{500, 2500, 0, false};
  CHECK(servoPulse(0, calibration, pulse) && pulse == 500);
  CHECK(servoPulse(180, calibration, pulse) && pulse == 2500);
  calibration.minUs = 499;
  CHECK(!servoPulse(90, calibration, pulse));
  calibration = ServoCalibration{1000, 2501, 0, false};
  CHECK(!servoPulse(90, calibration, pulse));
  calibration = ServoCalibration{1500, 1500, 0, false};
  CHECK(!servoPulse(90, calibration, pulse));
  calibration = ServoCalibration{2000, 1000, 0, false};
  CHECK(!servoPulse(90, calibration, pulse));
  calibration = ServoCalibration{};
  CHECK(!servoPulse(std::numeric_limits<float>::quiet_NaN(), calibration, pulse));
  CHECK(!servoPulse(std::numeric_limits<float>::infinity(), calibration, pulse));
  calibration.trimDegrees = std::numeric_limits<float>::quiet_NaN();
  CHECK(!servoPulse(90, calibration, pulse));
}

void testSettings()
{
  Settings settings;
  const char* error = nullptr;
  CHECK(validateSettings(settings, error));
  CHECK(setParameter(settings, "step_length", "42.5", error) && settings.stepLength == 42.5f);
  CHECK(setParameter(settings, "oscillator0_hz", "25000001", error));
  CHECK(settings.oscillatorHz(0) == 25000001 && settings.oscillatorHz(1) == 25000000);
  CHECK(!setParameter(settings, "step_length", "nan", error) && settings.stepLength == 42.5f);
  CHECK(!setParameter(settings, "timeout_ms", "1e3", error));
  CHECK(!setParameter(settings, "timeout_ms", "-1", error));
  CHECK(!setParameter(settings, "timeout_ms", "500.1", error));
  CHECK(!setParameter(settings, "timeout_ms", "42949672960", error));
  CHECK(!setParameter(settings, "timeout_ms", "500junk", error));
  CHECK(!setParameter(settings, "missing", "42", error));
  for (const FloatParameter& parameter : FLOAT_PARAMETERS)
  {
    Settings bad = settings;
    bad.*(parameter.member) = std::numeric_limits<float>::quiet_NaN();
    CHECK(!validateSettings(bad, error));
    bad.*(parameter.member) = parameter.maximum + 1;
    CHECK(!validateSettings(bad, error));
    bad.*(parameter.member) = parameter.minimum - 1;
    CHECK(!validateSettings(bad, error));
  }
  for (const IntegerParameter& parameter : INTEGER_PARAMETERS)
  {
    Settings bad = settings;
    bad.*(parameter.member) = parameter.maximum + 1;
    CHECK(!validateSettings(bad, error));
  }
  CHECK(!setParameter(settings, "coxa_min", "135", error));
  CHECK(!setParameter(settings, "stick_center", "10", error));
  CHECK(!setParameter(settings, "throttle_channel", "4", error));
  CHECK(!setParameter(settings, "home_z", "-200", error));
  Settings bad = settings;
  bad.origins[4] = {0, 0};
  CHECK(!validateSettings(bad, error));
  bad = settings;
  bad.servos[5][2].reversed = 255;
  CHECK(!validateSettings(bad, error));
  bad = settings;
  bad.servos[1][1].trimDegrees = 91;
  CHECK(!validateSettings(bad, error));

  settings.servos[5][2] = {800, 2200, 4.5f, 1};
  settings.origins[0] = {103, -60};
  const SettingsRecord saved = makeSettingsRecord(settings);
  Settings loaded;
  CHECK(decodeSettings(&saved, sizeof(saved), loaded, error));
  CHECK(loaded.stepLength == 42.5f && loaded.oscillator0Hz == 25000001);
  CHECK(loaded.servos[5][2].minUs == 800 && loaded.servos[5][2].reversed == 1);
  CHECK(loaded.origins[0].x == 103 && loaded.origins[0].y == -60);
  CHECK(!decodeSettings(&saved, sizeof(saved) - 1, loaded, error));
  CHECK(!decodeSettings(nullptr, sizeof(saved), loaded, error));
  SettingsRecord corrupt = saved;
  corrupt.version++;
  CHECK(!decodeSettings(&corrupt, sizeof(corrupt), loaded, error));
  corrupt = saved;
  corrupt.settings.stepLength = 999;
  CHECK(!decodeSettings(&corrupt, sizeof(corrupt), loaded, error));
  corrupt.checksum = settingsChecksum(corrupt.settings);
  CHECK(!decodeSettings(&corrupt, sizeof(corrupt), loaded, error));
  CHECK(loaded.stepLength == 42.5f); // Failed loads never partially apply.

  CHECK(setParameter(settings, "timeout_ms", "200", error));
  CHECK(setParameter(settings, "stick_center", "2000", error));
  CHECK(setParameter(settings, "stick_deadzone", "50", error));
  CHECK(setParameter(settings, "throttle_channel", "1", error));
  CHECK(setParameter(settings, "throttle_reverse", "1", error));
  ControlState state;
  ControlPacket packet = packetAt(100);
  packet.channels[1] = 2000;
  packet.channels[4] = 2000;
  CHECK(state.receive(packet, 100, settings) && state.armed);
  packet.receivedAtMs = 120;
  packet.channels[1] = 0;
  CHECK(state.receive(packet, 120, settings) && state.direction == -1);
  CHECK(!state.expire(319, settings.timeoutMs));
  CHECK(state.expire(320, settings.timeoutMs));
}

bool consoleCommand(const std::string& text, Command& command)
{
  std::string copy = text;
  const char* error = nullptr;
  return parseCommand(&copy[0], command, error);
}

void testConsole()
{
  Command command;
  const char* valid[] =
  {
    "help", "status", "show", "params", "save", "load", "defaults", "init", "scan",
    "off", "stop", "home", "radio", "center", "center all", "center lf",
    "servo rf coxa 90", "pulse 0 0 1500", "joints RR 90 100 110",
    "foot lf -10 0 20", "walk 1 -0.5 10000", "walk 0 0 100",
    "set step_height 40", "cal lm femur 500 2500 -90 1", "origin rf 100 -60"
  };
  for (const char* text : valid)
    CHECK(consoleCommand(text, command));
  CHECK(consoleCommand("SERVO LF TIBIA 45.5", command));
  CHECK(command.type == CommandType::Servo && command.leg == 5 && command.joint == 2);
  CHECK(command.values[0] == 45.5f);
  const char* invalid[] =
  {
    "", " ", "help extra", "unknown", "servo", "center bad", "servo 6 0 90",
    "servo rf 3 90", "servo rf coxa nan", "servo rf coxa inf", "servo rf coxa 181",
    "servo rf coxa -1", "pulse rf coxa 1500.1", "pulse rf coxa 2501",
    "walk 1 0", "walk 1 0 10001", "walk 1 0 99", "walk 1 0 -100",
    "walk 2 0 1000", "walk 0 -2 1000", "walk 1 0 1000 extra", "walk 1 0 99999999999999999",
    "cal rf coxa 2000 1000 0 0", "cal rf coxa 1000 2000 91 0",
    "cal rf coxa 1000 2000 0 2", "cal rf coxa 1000 2000 0 0 extra",
    "foot lf 151 0 0", "origin rf 501 0", "joints rf 90 90",
    "set step_height", "a b c d e f g h i"
  };
  for (const char* text : invalid)
    CHECK(!consoleCommand(text, command));

  ConsoleLine line;
  for (char character : std::string("help\r\n"))
  {
    const LineEvent event = line.push(character);
    if (character == '\r')
      CHECK(event == LineEvent::Ready && strcmp(line.data, "help") == 0);
    else
      CHECK(event == LineEvent::None);
  }
  for (size_t i = 0; i < sizeof(line.data) - 1; ++i)
    CHECK(line.push('a') == LineEvent::None);
  CHECK(line.push('a') == LineEvent::Overflow);
  for (char character : std::string("servo rf coxa 90\n"))
    CHECK(line.push(character) == LineEvent::None);
  CHECK(line.push('!') == LineEvent::Disable);
  CHECK(line.push('s') == LineEvent::None);
  CHECK(line.push('\n') == LineEvent::None);
  CHECK(line.push('\0') == LineEvent::Invalid);
  CHECK(line.push('\n') == LineEvent::None);
  CHECK(line.push('x') == LineEvent::None);
  CHECK(line.push('\b') == LineEvent::None);
  CHECK(line.push('h') == LineEvent::None);
  CHECK(line.push('\n') == LineEvent::Ready && strcmp(line.data, "h") == 0);

  TimedMotion motion{1, -1, UINT32_MAX - 10, 10000};
  CHECK(!motion.expired(uint32_t(motion.startedMs + 9999)));
  CHECK(motion.expired(uint32_t(motion.startedMs + 10000)));
}

int main()
{
  testProtocol();
  testControl();
  testServos();
  testSettings();
  testConsole();
  puts("PASS: protocol, failsafe, servo output, runtime settings, console parsing and saved records.");
}
