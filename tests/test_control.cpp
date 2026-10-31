#include "../HexapodControl.h"

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

int main()
{
  testProtocol();
  testControl();
  testServos();
  puts("PASS: CSV protocol, stick mapping, arming/failsafe, servo mapping and PWM conversion.");
}
