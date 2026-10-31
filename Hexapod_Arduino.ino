// Özgür Toprak Önsoy, 11.2025, Eskisehir, Turkiye
// Emre Kalem, 11.2025, Eskisehir, Turkiye

/**
MIT License

Copyright 2025 Emre Kalem

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), 
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, 
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND , EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include <math.h>
#include "HexapodConsole.h"

#define POW2(x)       ((x)*(x))

using namespace Hexapod;

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr int SERVO_OE = 23;
constexpr uint8_t DRIVER_ADDRESSES[DRIVER_COUNT] = {0x40, 0x41};
Settings settings;
Preferences preferences;
ConsoleLine consoleLine;
TimedMotion manualMotion;
constexpr uint32_t SERVO_TEST_STEP_MS = 1000;
constexpr float SERVO_TEST_ANGLES[] = {90.0f, 85.0f, 95.0f, 90.0f};
constexpr size_t SERVO_TEST_STEP_COUNT = sizeof(SERVO_TEST_ANGLES) / sizeof(SERVO_TEST_ANGLES[0]);
struct ServoTestState
{
  size_t servoIndex = 0;
  size_t step = 0;
  uint32_t startedMs = 0;
};
ServoTestState servoTest;
enum class Mode { Off, Pose, Home, Radio, Walk, ServoTest, ServoTestPaused };
Mode mode = Mode::Off;
bool servosReady = false;
bool radioReady = false;
bool radioInitialized = false;
bool wireReady = false;
bool storageReady = false;
bool settingsDirty = false;

Adafruit_PWMServoDriver pwm[DRIVER_COUNT] =
{
  Adafruit_PWMServoDriver(DRIVER_ADDRESSES[0]),
  Adafruit_PWMServoDriver(DRIVER_ADDRESSES[1])
};
uint8_t pwmPrescale[DRIVER_COUNT] = {};
uint16_t lastPulses[LEG_COUNT][JOINT_COUNT] = {};
QueueHandle_t controlQueue = nullptr;
std::atomic<uint32_t> rejectedPackets{0};
ControlState control;
ControlPacket lastPacket;
bool havePacket = false;
uint32_t lastUpdateMs = 0;

struct Vector
{
  float x;
  float y;
  float z;

  Vector(float _x = 0.0, float _y = 0.0, float _z = 0.0)
    : x(_x),
    y(_y),
    z(_z)
  {
  }
};

struct Leg
{
  Vector home;
  Vector origin;
  Vector target;

};

Leg legs[LEG_COUNT];
float gaitPhase = 0;

void calculate_ik(const Leg& leg, Vector& res);
Vector get_foot_target(float t, size_t legIndex, float direction, float rotation);
bool set_foot_positions(float direction, float rotation);
void read_control();
void read_console();

void disable_outputs()
{
  digitalWrite(SERVO_OE, HIGH);
  mode = Mode::Off;
  control = ControlState{};
  servoTest = ServoTestState{};
  memset(lastPulses, 0, sizeof(lastPulses));
}

bool servo_error(const char* message)
{
  disable_outputs();
  servosReady = false;
  Serial.printf("ERROR: %s Outputs disabled; fix wiring/settings, then use init.\n", message);
  return false;
}

bool require_off()
{
  if (mode == Mode::Off)
    return true;
  Serial.println("ERROR: use off first (support the robot; torque will be released).");
  return false;
}

void rebuild_geometry()
{
  gaitPhase = 0;
  const float homeReach = settings.coxaLength +
      (settings.femurLength + settings.tibiaLength) * cos(M_PI / 4);
  for (size_t i = 0; i < LEG_COUNT; ++i)
  {
    legs[i].origin = Vector(settings.origins[i].x, settings.origins[i].y,
                            atan2(settings.origins[i].y, settings.origins[i].x));
    legs[i].home = Vector(legs[i].origin.x + homeReach * cos(legs[i].origin.z),
                          legs[i].origin.y + homeReach * sin(legs[i].origin.z), settings.homeZ);
    legs[i].target = legs[i].home;
  }
}

void on_control_received(const esp_now_recv_info_t*, const uint8_t* data, int length)
{
  ControlPacket packet;
  if (length <= 0 || !parseControlPacket(data, size_t(length), packet))
  {
    rejectedPackets.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  packet.receivedAtMs = millis();
  // The Wi-Fi task only publishes data; all I2C work stays in loop().
  if (xQueueOverwrite(controlQueue, &packet) != pdPASS)
    rejectedPackets.fetch_add(1, std::memory_order_relaxed);
}

bool init_control()
{
  radioReady = false;
  if (controlQueue == nullptr)
    controlQueue = xQueueCreate(1, sizeof(ControlPacket));
  if (controlQueue == nullptr || !WiFi.mode(WIFI_STA))
  {
    Serial.println("ERROR: cannot allocate radio queue/start Wi-Fi.");
    return false;
  }
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
      esp_wifi_set_channel(uint8_t(settings.radioChannel), WIFI_SECOND_CHAN_NONE) != ESP_OK)
  {
    Serial.println("ERROR: cannot configure ESP-NOW channel/power saving.");
    return false;
  }
  Serial.printf("Receiver STA MAC: %s; ESP-NOW channel: %u\n",
                WiFi.macAddress().c_str(), unsigned(settings.radioChannel));
  if (!radioInitialized)
  {
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("ERROR: cannot initialize ESP-NOW.");
      return false;
    }
    radioInitialized = true;
  }
  if (esp_now_register_recv_cb(on_control_received) != ESP_OK)
  {
    Serial.println("ERROR: cannot register ESP-NOW receiver.");
    return false;
  }
  xQueueReset(controlQueue);
  control = ControlState{};
  havePacket = false;
  radioReady = true;
  return true;
}

bool write_pulse(size_t legIndex, size_t jointIndex, uint16_t pulseUs)
{
  if (!servosReady)
  {
    Serial.println("ERROR: servo drivers not ready; use init.");
    return false;
  }
  const size_t board = servoBoard(legIndex);
  const uint8_t channel = servoChannel(legIndex, jointIndex);
  const uint16_t ticks = pulseToTicks(pulseUs, settings.oscillatorHz(board), pwmPrescale[board]);
  if (ticks == 0 || ticks >= 4096)
    return servo_error("Servo pulse outside PCA9685 range.");
  if (pwm[board].setPWM(channel, 0, ticks) != 0)
  {
    Serial.printf("PCA9685 write failed: address 0x%02X, channel %u\n",
                  DRIVER_ADDRESSES[board], channel);
    return servo_error("I2C servo write failed.");
  }
  lastPulses[legIndex][jointIndex] = pulseUs;
  return true;
}

bool write_servo(size_t legIndex, size_t jointIndex, float angle)
{
  uint16_t pulseUs = 0;
  if (!servoPulse(angle, settings.servos[legIndex][jointIndex], pulseUs))
    return servo_error("Invalid servo angle or calibration.");
  return write_pulse(legIndex, jointIndex, pulseUs);
}

bool clear_servo_outputs()
{
  for (size_t board = 0; board < DRIVER_COUNT; ++board)
    for (uint8_t channel = 0; channel < 16; ++channel)
      if (pwm[board].setPWM(channel, 0, 4096) != 0)
        return servo_error("Cannot clear PCA9685 outputs.");
  memset(lastPulses, 0, sizeof(lastPulses));
  return true;
}

bool init_servos()
{
  servosReady = false;
  if (!wireReady)
    wireReady = Wire.begin(I2C_SDA, I2C_SCL);
  if (!wireReady || !Wire.setClock(settings.i2cHz))
    return servo_error("Cannot configure I2C.");
  Wire.setTimeOut(20);

  for (size_t board = 0; board < DRIVER_COUNT; ++board)
  {
    if (!pwm[board].begin())
    {
      Serial.printf("Missing PCA9685 at 0x%02X\n", DRIVER_ADDRESSES[board]);
      return servo_error("Check PCA9685 power, wiring and address jumpers.");
    }
    pwm[board].setOscillatorFrequency(settings.oscillatorHz(board));
    pwm[board].setPWMFreq(settings.servoHz);
    pwmPrescale[board] = pwm[board].readPrescale();
    const uint8_t expected = uint8_t(lroundf(
        settings.oscillatorHz(board) / (4096.0f * settings.servoHz)) - 1);
    if (pwmPrescale[board] != expected)
      return servo_error("PCA9685 frequency configuration failed.");
  }
  if (!clear_servo_outputs())
    return false;
  servosReady = true;
  Serial.println("PCA9685 drivers ready; outputs remain disabled.");
  return true;
}

bool load_settings(Settings& loaded)
{
  if (!storageReady)
  {
    Serial.println("ERROR: flash settings storage unavailable.");
    return false;
  }
  if (!preferences.isKey("settings"))
  {
    Serial.println("No saved settings.");
    return false;
  }
  SettingsRecord record;
  if (preferences.getBytesLength("settings") != sizeof(record) ||
      preferences.getBytes("settings", &record, sizeof(record)) != sizeof(record))
  {
    Serial.println("ERROR: saved settings size/read failure; RAM settings unchanged.");
    return false;
  }
  const char* error = nullptr;
  if (!decodeSettings(&record, sizeof(record), loaded, error))
  {
    Serial.printf("ERROR: %s; RAM settings unchanged.\n", error);
    return false;
  }
  Serial.println("Settings loaded from flash; outputs remain disabled.");
  return true;
}

void save_settings()
{
  if (!storageReady)
  {
    Serial.println("ERROR: flash settings storage unavailable.");
    return;
  }
  const char* error = nullptr;
  if (!validateSettings(settings, error))
  {
    Serial.printf("ERROR: cannot save invalid settings: %s\n", error);
    return;
  }
  const SettingsRecord record = makeSettingsRecord(settings);
  SettingsRecord readBack;
  if (preferences.putBytes("settings", &record, sizeof(record)) != sizeof(record) ||
      preferences.getBytes("settings", &readBack, sizeof(readBack)) != sizeof(readBack) ||
      memcmp(&record, &readBack, sizeof(record)) != 0)
  {
    Serial.println("ERROR: settings save/read-back verification failed.");
    return;
  }
  settingsDirty = false;
  Serial.println("Settings saved to flash and verified.");
}

void apply_settings(const Settings& candidate)
{
  if (settings.oscillator0Hz != candidate.oscillator0Hz ||
      settings.oscillator1Hz != candidate.oscillator1Hz ||
      settings.servoHz != candidate.servoHz || settings.i2cHz != candidate.i2cHz)
    servosReady = false;
  if (settings.radioChannel != candidate.radioChannel)
    radioReady = false;
  settings = candidate;
  settingsDirty = true;
  control = ControlState{};
  rebuild_geometry();
  if (!servosReady || !radioReady)
    Serial.println("Hardware configuration pending: use init before enabling affected outputs/radio.");
}

bool prepare_pose()
{
  if (mode != Mode::Off && mode != Mode::Pose)
    return require_off();
  if (!servosReady)
  {
    Serial.println("ERROR: servo drivers not ready; use init.");
    return false;
  }
  return mode == Mode::Pose || clear_servo_outputs();
}

void enable_pose()
{
  mode = Mode::Pose;
  digitalWrite(SERVO_OE, LOW);
  Serial.println("Pose applied; ESP-NOW ignored. Other previously tested joints keep holding.");
}

void print_servo_test_step()
{
  const size_t leg = servoTest.servoIndex / JOINT_COUNT;
  const size_t joint = servoTest.servoIndex % JOINT_COUNT;
  Serial.printf("Servo test %u/%u: %s %s, address 0x%02X channel %u, nominal %.0f deg (%u us).\n",
      unsigned(servoTest.servoIndex + 1), unsigned(LEG_COUNT * JOINT_COUNT),
      LEG_NAMES[leg], JOINT_NAMES[joint], DRIVER_ADDRESSES[servoBoard(leg)],
      servoChannel(leg, joint), SERVO_TEST_ANGLES[servoTest.step], lastPulses[leg][joint]);
}

void print_servo_test_pause()
{
  const size_t leg = servoTest.servoIndex / JOINT_COUNT;
  Serial.printf("Leg %s test complete; outputs disabled. Paused: retest repeats this leg; next %s.\n",
      LEG_NAMES[leg], leg + 1 < LEG_COUNT ? "tests the next leg" : "finishes the sequence");
}

bool apply_servo_test_step()
{
  const size_t leg = servoTest.servoIndex / JOINT_COUNT;
  const size_t joint = servoTest.servoIndex % JOINT_COUNT;
  if (!write_servo(leg, joint, SERVO_TEST_ANGLES[servoTest.step]))
    return false;
  mode = Mode::ServoTest;
  digitalWrite(SERVO_OE, LOW);
  // Start a fresh hold after each write; delayed loops must not skip angles.
  servoTest.startedMs = millis();
  return true;
}

void start_servo_test()
{
  if (!require_off() || !prepare_pose())
    return;
  servoTest = ServoTestState{};
  if (!apply_servo_test_step())
    return;
  Serial.println("Servo test started: one joint at a time, 90/85/95/90 deg, 1000 ms each."
                 " Pauses after each leg: retest repeats it, next advances."
                 " Support the robot; stop holds the current joint, !/off disables outputs.");
  print_servo_test_step();
}

void resume_servo_test(bool retest)
{
  if (mode != Mode::ServoTestPaused)
  {
    Serial.println("ERROR: next/retest requires testservos paused after a leg.");
    return;
  }
  if (retest)
    servoTest.servoIndex = (servoTest.servoIndex / JOINT_COUNT) * JOINT_COUNT;
  else if (++servoTest.servoIndex == LEG_COUNT * JOINT_COUNT)
  {
    disable_outputs();
    Serial.println("Servo test complete; outputs disabled. Radio remains ignored.");
    return;
  }
  servoTest.step = 0;
  if (apply_servo_test_step())
    print_servo_test_step();
}

void update_servo_test(uint32_t now)
{
  if (mode != Mode::ServoTest || uint32_t(now - servoTest.startedMs) < SERVO_TEST_STEP_MS)
    return;
  const size_t leg = servoTest.servoIndex / JOINT_COUNT;
  const size_t joint = servoTest.servoIndex % JOINT_COUNT;
  if (servoTest.step + 1 == SERVO_TEST_STEP_COUNT)
  {
    digitalWrite(SERVO_OE, HIGH);
    if (pwm[servoBoard(leg)].setPWM(servoChannel(leg, joint), 0, 4096) != 0)
    {
      servo_error("Cannot disable tested servo.");
      return;
    }
    lastPulses[leg][joint] = 0;
    if (joint + 1 == JOINT_COUNT)
    {
      mode = Mode::ServoTestPaused;
      print_servo_test_pause();
      return;
    }
    ++servoTest.servoIndex;
    servoTest.step = 0;
  }
  else
    ++servoTest.step;
  if (apply_servo_test_step())
    print_servo_test_step();
}

bool enter_home()
{
  if (!servosReady)
  {
    Serial.println("ERROR: servo drivers not ready; use init.");
    return false;
  }
  if (mode == Mode::Off || mode == Mode::Pose || mode == Mode::ServoTest ||
      mode == Mode::ServoTestPaused)
  {
    digitalWrite(SERVO_OE, HIGH);
    rebuild_geometry();
    if (!set_foot_positions(0, 0))
      return false;
    digitalWrite(SERVO_OE, LOW);
  }
  control = ControlState{};
  mode = Mode::Home;
  return true;
}

void print_status()
{
  const char* names[] = {"off", "pose", "home", "radio", "walk", "testservos", "testservos-paused"};
  Serial.printf("mode=%s OE=%s drivers=%s radio=%s unsaved=%s MAC=%s\n",
      names[size_t(mode)], mode == Mode::Off || mode == Mode::ServoTestPaused ? "HIGH" : "LOW",
      servosReady ? "ready" : "init-needed", radioReady ? "ready" : "init-needed",
      settingsDirty ? "yes" : "no", WiFi.macAddress().c_str());
  Serial.printf("armed=%u direction=%.2f rotation=%.2f configured-channel=%u\n", control.armed,
      mode == Mode::Walk ? manualMotion.direction : control.direction,
      mode == Mode::Walk ? manualMotion.rotation : control.rotation, unsigned(settings.radioChannel));
  if (havePacket)
  {
    Serial.printf("last valid RX age=%lu ms CSV=", static_cast<unsigned long>(millis() - lastPacket.receivedAtMs));
    for (size_t i = 0; i < CONTROL_CHANNEL_COUNT; ++i)
      Serial.printf(i + 1 == CONTROL_CHANNEL_COUNT ? "%u\n" : "%u,", lastPacket.channels[i]);
  }
  else
    Serial.println("No valid RX packet since initialization.");
  if (mode == Mode::ServoTest)
    print_servo_test_step();
  else if (mode == Mode::ServoTestPaused)
    print_servo_test_pause();
  if (mode == Mode::Off || mode == Mode::Pose || mode == Mode::ServoTest ||
      mode == Mode::ServoTestPaused)
    for (size_t leg = 0; leg < LEG_COUNT; ++leg)
      Serial.printf("%s commanded us (0=off): %u %u %u\n", LEG_NAMES[leg],
                    lastPulses[leg][0], lastPulses[leg][1], lastPulses[leg][2]);
}

void print_help()
{
  Serial.println(
      "Commands (newline or CRLF, 115200 baud):\n"
      "  help | status | params | show\n"
      "  off | ! (immediate disable; finish with newline) | stop (return home if walking)\n"
      "  center [all|LEG] | servo LEG JOINT ANGLE | pulse LEG JOINT US\n"
      "  testservos (while off; one joint at a time, 90/85/95/90 deg, 1000 ms each)\n"
      "  next | retest (only at the pause after each leg; outputs disabled while paused)\n"
      "  joints LEG COXA FEMUR TIBIA | foot LEG DX DY DZ (offset mm from home)\n"
      "  home | radio | walk DIRECTION ROTATION DURATION_MS (100..10000)\n"
      "  set NAME VALUE | cal LEG JOINT MIN_US MAX_US TRIM REVERSE\n"
      "  origin LEG X Y | save | load | defaults | init | scan\n"
      "LEG=rf/rm/rr/lr/lm/lf or 0..5; JOINT=coxa/femur/tibia or 0..2.\n"
      "Use off before configuration, storage, scan, init, help, params or show.\n"
      "Direct joint tests require off/pose. walk/home/radio explicitly take control.\n"
      "next advances/finishes testservos; retest repeats the paused leg.\n"
      "stop cancels testservos holding the active joint or staying off if paused.\n"
      "Only save writes settings. Boot always stays off; !/off releases torque.");
}

void show_settings(bool ranges)
{
  for (const FloatParameter& parameter : FLOAT_PARAMETERS)
  {
    if (ranges)
      Serial.printf("%s: %.9g..%.9g\n", parameter.name, parameter.minimum, parameter.maximum);
    else
      Serial.printf("set %s %.9g\n", parameter.name, settings.*(parameter.member));
  }
  for (const IntegerParameter& parameter : INTEGER_PARAMETERS)
  {
    if (ranges)
      Serial.printf("%s: %lu..%lu (integer)\n", parameter.name,
                    static_cast<unsigned long>(parameter.minimum), static_cast<unsigned long>(parameter.maximum));
    else
      Serial.printf("set %s %lu\n", parameter.name,
                    static_cast<unsigned long>(settings.*(parameter.member)));
  }
  if (ranges)
    return;
  for (size_t leg = 0; leg < LEG_COUNT; ++leg)
  {
    Serial.printf("origin %s %.9g %.9g\n", LEG_NAMES[leg], settings.origins[leg].x, settings.origins[leg].y);
    for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
    {
      const ServoCalibration& calibration = settings.servos[leg][joint];
      Serial.printf("cal %s %s %u %u %.9g %u\n", LEG_NAMES[leg], JOINT_NAMES[joint],
          calibration.minUs, calibration.maxUs, calibration.trimDegrees, calibration.reversed);
    }
  }
}

void execute_command(const Command& command)
{
  if (command.type == CommandType::Off)
  {
    disable_outputs();
    Serial.println("Outputs disabled.");
    return;
  }
  if (command.type == CommandType::Status)
  {
    print_status();
    return;
  }
  if (command.type == CommandType::Stop)
  {
    if (mode == Mode::Radio || mode == Mode::Walk)
      mode = Mode::Home;
    else if (mode == Mode::ServoTest)
    {
      mode = Mode::Pose;
      servoTest = ServoTestState{};
    }
    else if (mode == Mode::ServoTestPaused)
    {
      disable_outputs();
      Serial.println("Servo test cancelled; outputs remain disabled.");
      return;
    }
    control = ControlState{};
    Serial.println("Motion stopped; current pose/home held unless outputs were already off.");
    return;
  }
  if (command.type == CommandType::TestServos)
  {
    start_servo_test();
    return;
  }
  if (command.type == CommandType::Next || command.type == CommandType::Retest)
  {
    resume_servo_test(command.type == CommandType::Retest);
    return;
  }
  if (command.type == CommandType::Home || command.type == CommandType::Radio ||
      command.type == CommandType::Walk)
  {
    if (command.type == CommandType::Radio && !radioReady)
    {
      Serial.println("ERROR: radio not ready; use off then init.");
      return;
    }
    if (!enter_home())
      return;
    if (command.type == CommandType::Radio)
    {
      xQueueReset(controlQueue);
      mode = Mode::Radio;
      Serial.println("ESP-NOW mode: center sticks to arm. Serial tests do not resume radio automatically.");
    }
    else if (command.type == CommandType::Walk)
    {
      manualMotion = {command.values[0], command.values[1], millis(), command.durationMs};
      mode = Mode::Walk;
      Serial.println("Timed walking test started; stop returns home, ! disables outputs.");
    }
    else
      Serial.println("Home stance enabled; radio ignored.");
    return;
  }
  if (command.type == CommandType::Center || command.type == CommandType::Servo ||
      command.type == CommandType::Pulse || command.type == CommandType::Joints ||
      command.type == CommandType::Foot)
  {
    Vector angles;
    if (command.type == CommandType::Pulse)
    {
      const ServoCalibration& calibration = settings.servos[command.leg][command.joint];
      if (command.values[0] < calibration.minUs || command.values[0] > calibration.maxUs)
      {
        Serial.println("ERROR: pulse outside this servo's calibrated min/max; use cal while off.");
        return;
      }
    }
    if (command.type == CommandType::Foot)
    {
      Leg testLeg = legs[command.leg];
      testLeg.target = Vector(testLeg.home.x + command.values[0], testLeg.home.y + command.values[1],
                              testLeg.home.z + command.values[2]);
      const float forward = hypotf(testLeg.target.x - testLeg.origin.x,
                                    testLeg.target.y - testLeg.origin.y) - settings.coxaLength;
      const float distance = hypotf(forward, testLeg.target.z);
      if (distance <= fabsf(settings.femurLength - settings.tibiaLength) + 0.001f ||
          distance >= settings.femurLength + settings.tibiaLength - 0.001f)
      {
        Serial.println("ERROR: foot target outside leg reach.");
        return;
      }
      calculate_ik(testLeg, angles);
      angles = Vector(90 + angles.x, 90 + angles.y, fabsf(angles.z));
      if (!isfinite(angles.x) || !isfinite(angles.y) || !isfinite(angles.z) ||
          angles.x < settings.coxaMin || angles.x > settings.coxaMax ||
          angles.y < 0 || angles.y > 180 || angles.z < 0 || angles.z > 180)
      {
        Serial.println("ERROR: foot target outside joint angle limits.");
        return;
      }
    }
    if (!prepare_pose())
      return;
    if (command.type == CommandType::Center)
    {
      for (size_t leg = 0; leg < LEG_COUNT; ++leg)
        if (command.leg == LEG_COUNT || command.leg == leg)
          for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
            if (!write_servo(leg, joint, 90))
              return;
    }
    else if (command.type == CommandType::Servo)
    {
      if (!write_servo(command.leg, command.joint, command.values[0]))
        return;
    }
    else if (command.type == CommandType::Pulse)
    {
      if (!write_pulse(command.leg, command.joint, uint16_t(command.values[0])))
        return;
    }
    else
    {
      const float values[] = {angles.x, angles.y, angles.z};
      for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
        if (!write_servo(command.leg, joint,
                         command.type == CommandType::Foot ? values[joint] : command.values[joint]))
          return;
    }
    enable_pose();
    return;
  }

  if (!require_off())
    return;
  switch (command.type)
  {
    case CommandType::Help: print_help(); break;
    case CommandType::Show: show_settings(false); break;
    case CommandType::Params: show_settings(true); break;
    case CommandType::Save: save_settings(); break;
    case CommandType::Init:
      init_control();
      init_servos();
      break;
    case CommandType::Scan:
      if (!wireReady)
        Serial.println("ERROR: I2C not initialized; use init.");
      else
      {
        unsigned found = 0;
        for (uint8_t address = 8; address < 120; ++address)
        {
          Wire.beginTransmission(address);
          const uint8_t result = Wire.endTransmission();
          if (result == 0)
          {
            Serial.printf("I2C device: 0x%02X\n", address);
            ++found;
          }
          else if (result != 2)
          {
            Serial.printf("ERROR: I2C scan failed at 0x%02X, code %u\n", address, result);
            return;
          }
        }
        Serial.printf("Scan complete: %u responding address(es).\n", found);
      }
      break;
    case CommandType::Load:
    {
      Settings loaded;
      if (load_settings(loaded))
      {
        apply_settings(loaded);
        settingsDirty = false;
      }
      break;
    }
    case CommandType::Defaults:
      apply_settings(Settings{});
      Serial.println("Factory defaults in RAM only; save to replace saved settings.");
      break;
    case CommandType::Set:
    case CommandType::Calibrate:
    case CommandType::Origin:
    {
      Settings candidate = settings;
      const char* error = nullptr;
      if (command.type == CommandType::Set)
      {
        if (!setParameter(candidate, command.name, command.text, error))
        {
          Serial.printf("ERROR: %s\n", error);
          return;
        }
      }
      else if (command.type == CommandType::Calibrate)
        candidate.servos[command.leg][command.joint] =
            {uint16_t(command.values[0]), uint16_t(command.values[1]), command.values[2], uint8_t(command.values[3])};
      else
        candidate.origins[command.leg] = {command.values[0], command.values[1]};
      if (!validateSettings(candidate, error))
      {
        Serial.printf("ERROR: %s\n", error);
        return;
      }
      apply_settings(candidate);
      Serial.println("Setting updated in RAM; use save to persist.");
      break;
    }
    default: Serial.println("ERROR: unsupported command."); break;
  }
}

void read_console()
{
  for (size_t budget = 0; budget < 64 && Serial.available(); ++budget)
  {
    const LineEvent event = consoleLine.push(char(Serial.read()));
    if (event == LineEvent::Disable)
    {
      disable_outputs();
      Serial.println("Outputs disabled. Finish the line before the next command.");
      return;
    }
    if (event == LineEvent::Overflow || event == LineEvent::Invalid)
    {
      Serial.println("ERROR: invalid/overlong line discarded; finish with newline.");
      return;
    }
    if (event == LineEvent::Ready)
    {
      Command command;
      const char* error = nullptr;
      if (parseCommand(consoleLine.data, command, error))
        execute_command(command);
      else
        Serial.printf("ERROR: %s\n", error);
      return;
    }
  }
}

void setup()
{
  pinMode(SERVO_OE, OUTPUT);
  disable_outputs();
  Serial.begin(115200);
  Serial.println("Hexapod serial console. Boot is always OFF; type help.");
  storageReady = preferences.begin("hexapod", false);
  if (!load_settings(settings))
    Serial.println("Using factory defaults in RAM; outputs remain disabled.");
  rebuild_geometry();
  init_control();
  init_servos();
  lastUpdateMs = millis();
}

void loop()
{
  read_control();
  if (mode == Mode::Walk && manualMotion.expired(millis()))
  {
    mode = Mode::Home;
    Serial.println("Walking test complete; returning home. Radio remains ignored.");
  }
  read_console();
  const uint32_t now = millis();
  update_servo_test(now);
  if (uint32_t(now - lastUpdateMs) >= settings.updateMs)
  {
    lastUpdateMs = now;
    if (mode == Mode::Radio)
      set_foot_positions(control.direction, control.rotation);
    else if (mode == Mode::Walk)
      set_foot_positions(manualMotion.direction, manualMotion.rotation);
    else if (mode == Mode::Home)
      set_foot_positions(0, 0);
  }
  delay(1);
}

void read_control()
{
  const uint32_t now = millis();
  if (mode == Mode::Radio && control.expire(now, settings.timeoutMs))
    Serial.println("ESP-NOW timeout: returning home; center sticks to re-arm.");

  ControlPacket packet;
  if (controlQueue != nullptr && xQueueReceive(controlQueue, &packet, 0) == pdTRUE)
  {
    lastPacket = packet;
    havePacket = true;
    const bool wasArmed = control.armed;
    if (mode == Mode::Radio)
    {
      if (!control.receive(packet, millis(), settings))
        Serial.println("Discarded stale ESP-NOW packet.");
      else if (!wasArmed && control.armed)
        Serial.println("ESP-NOW control armed.");
    }
  }

  static uint32_t lastReportMs = 0;
  if (uint32_t(now - lastReportMs) >= 1000)
  {
    lastReportMs = now;
    const uint32_t rejected = rejectedPackets.exchange(0, std::memory_order_relaxed);
    if (rejected != 0)
      Serial.printf("Rejected %lu ESP-NOW packet(s); expected ten valid CSV fields.\n",
                    static_cast<unsigned long>(rejected));
  }
}

void calculate_ik(const Leg& leg, Vector& res)
{
  Vector relativeTarget(
    (leg.target.x - leg.origin.x),
    (leg.target.y - leg.origin.y),
    leg.target.z
  );

  const float c = cos(-leg.origin.z);
  const float s = sin(-leg.origin.z);

  const float tempX = relativeTarget.x;
  const float tempY = relativeTarget.y;

  relativeTarget.x = tempX * c - tempY * s;
  relativeTarget.y = tempX * s + tempY * c;

  const float l_xy = sqrt(POW2(relativeTarget.x) + POW2(relativeTarget.y));
  const float l_forward = l_xy - settings.coxaLength;
  float D = sqrt(POW2(l_forward) + POW2(relativeTarget.z));

  D = constrain(D, fabsf(settings.femurLength - settings.tibiaLength) + 0.001f,
                settings.femurLength + settings.tibiaLength - 0.001f);

  const float kneeCos = constrain((POW2(settings.femurLength) + POW2(settings.tibiaLength) - POW2(D)) /
                                 (2.0 * settings.femurLength * settings.tibiaLength), -1.0, 1.0);
  res.z = (acos(kneeCos) - M_PI) * 180.0 / M_PI;

  const float beta1 = atan2(relativeTarget.z, l_forward);
  const float hipCos = constrain((POW2(settings.femurLength) + POW2(D) - POW2(settings.tibiaLength)) /
                                (2.0 * settings.femurLength * D), -1.0, 1.0);
  const float beta2 = acos(hipCos);
  res.y = (beta1 + beta2) * 180.0 / M_PI;

  res.x = atan2(relativeTarget.y, relativeTarget.x) * 180.0 / M_PI;
}

Vector get_foot_target(float t, size_t legIndex, float direction, float rotation)
{
  float phase = fmod(t, 1.0);
  if ((legIndex % 2) != 0) // Legs 1, 3, 5
    phase = fmod(phase + 0.5, 1.0);
 

  Vector home_pos = legs[legIndex].home;
  
  if (direction == 0 && rotation == 0)
    return home_pos;
 
  float step_x_move = 0.0;
  float step_rot_rad = 0.0;
  float step_z_move = 0.0;

  if (phase < 0.5)
  {
    float phase_norm = phase * 2.0;
    step_x_move = direction * (0.5 - phase_norm) * settings.stepLength;
    step_rot_rad = rotation * (0.5 - phase_norm) * settings.rotationStep * M_PI / 180.0;
    step_z_move = 0;
  }
  else
  {
    float phase_norm = (phase - 0.5) * 2.0; // 0 -> 1
    step_x_move = direction * (phase_norm - 0.5) * settings.stepLength;
    step_rot_rad = rotation * (phase_norm - 0.5) * settings.rotationStep * M_PI / 180.0;
    step_z_move = sin(phase_norm * M_PI) * settings.stepHeight;
  }

  float c_rot = cos(step_rot_rad);
  float s_rot = sin(step_rot_rad);

  float rotated_home_x = home_pos.x * c_rot - home_pos.y * s_rot;
  float rotated_home_y = home_pos.x * s_rot + home_pos.y * c_rot;

  Vector res;
  res.x = rotated_home_x + step_x_move;
  res.y = rotated_home_y;
  res.z = home_pos.z + step_z_move;

  return res;
}

bool set_foot_positions(float direction, float rotation)
{
  gaitPhase = fmod(gaitPhase + settings.phaseStep, 1.0);

  Vector angles;
  for (size_t i = 0; i < LEG_COUNT; ++i)
  {
    const Vector goalTarget = get_foot_target(gaitPhase, i, direction, rotation);

    legs[i].target.x += (goalTarget.x - legs[i].target.x) * settings.smoothing;
    legs[i].target.y += (goalTarget.y - legs[i].target.y) * settings.smoothing;
    legs[i].target.z += (goalTarget.z - legs[i].target.z) * settings.smoothing;

    calculate_ik(legs[i], angles);

    float ax = 90 + angles.x;
    float ay = 90 + angles.y;
    float az = fabs(angles.z);

    ax = constrain(ax, settings.coxaMin, settings.coxaMax);

    if (!write_servo(i, 0, ax) || !write_servo(i, 1, ay) || !write_servo(i, 2, az))
      return false;
  }
  return true;
}
