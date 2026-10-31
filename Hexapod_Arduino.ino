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
#include "HexapodWeb.h"

#define POW2(x)       ((x)*(x))

using namespace Hexapod;

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr int SERVO_OE = 23;
constexpr uint8_t DRIVER_ADDRESSES[DRIVER_COUNT] = {0x40, 0x41};
Settings settings;
Preferences preferences;
ConsoleLine consoleLine;
ConsoleOutput console;
WebTransport webTransport;
WebMotion webMotion;
uint32_t webGeneration = 0;
bool wifiReady = false;
bool apActive = false;
bool apRetryPending = false;
uint32_t lastApAttemptMs = 0;
char apName[32] = {};
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
enum class Mode { Off, Pose, Home, Radio, Walk, ServoTest, ServoTestPaused, Web };
constexpr const char* MODE_NAMES[] = {"off", "pose", "home", "radio", "walk",
                                    "testservos", "testservos-paused", "web"};
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
uint16_t targetPulses[LEG_COUNT][JOINT_COUNT] = {};
constexpr size_t SERVO_COUNT = LEG_COUNT * JOINT_COUNT;
constexpr uint32_t SERVO_STAGGER_MS = 250;
bool posePending = false;
size_t nextServo = 0;
uint32_t lastServoWriteMs = 0;
uint32_t servoWriteWaitMs = 0;
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
void update_network();
void read_web();

void disable_outputs()
{
  digitalWrite(SERVO_OE, HIGH);
  mode = Mode::Off;
  ++webGeneration;
  webMotion = WebMotion{};
  control = ControlState{};
  servoTest = ServoTestState{};
  posePending = false;
  nextServo = 0;
  memset(lastPulses, 0, sizeof(lastPulses));
  memset(targetPulses, 0, sizeof(targetPulses));
}

bool servo_error(const char* message)
{
  disable_outputs();
  servosReady = false;
  console.printf("ERROR: %s Outputs disabled; fix wiring/settings, then use init.\n", message);
  return false;
}

bool require_off()
{
  if (mode == Mode::Off)
    return true;
  console.println("ERROR: use off first (support the robot; torque will be released).");
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
  wifiReady = false;
  havePacket = false;
  if (apActive)
  {
    if (!WiFi.softAPdisconnect(true))
    {
      console.println("ERROR: cannot stop fallback AP; retry init.");
      return false;
    }
    apActive = false;
    ++webGeneration;
  }
  apRetryPending = false;
  if (controlQueue == nullptr)
    controlQueue = xQueueCreate(1, sizeof(ControlPacket));
  if (controlQueue == nullptr || !WiFi.mode(WIFI_STA))
  {
    console.println("ERROR: cannot allocate radio queue/start Wi-Fi.");
    return false;
  }
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
      esp_wifi_set_channel(uint8_t(settings.radioChannel), WIFI_SECOND_CHAN_NONE) != ESP_OK)
  {
    console.println("ERROR: cannot configure ESP-NOW channel/power saving.");
    return false;
  }
  wifiReady = true;
  console.printf("Receiver STA MAC: %s; ESP-NOW channel: %u\n",
                WiFi.macAddress().c_str(), unsigned(settings.radioChannel));
  if (!radioInitialized)
  {
    if (esp_now_init() != ESP_OK)
    {
      console.println("ERROR: cannot initialize ESP-NOW.");
      return false;
    }
    radioInitialized = true;
  }
  if (esp_now_register_recv_cb(on_control_received) != ESP_OK)
  {
    console.println("ERROR: cannot register ESP-NOW receiver.");
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
    console.println("ERROR: servo drivers not ready; use init.");
    return false;
  }
  const size_t board = servoBoard(legIndex);
  const uint8_t channel = servoChannel(legIndex, jointIndex);
  const uint16_t ticks = pulseToTicks(pulseUs, settings.oscillatorHz(board), pwmPrescale[board]);
  if (ticks == 0 || ticks >= 4096)
    return servo_error("Servo pulse outside PCA9685 range.");
  // Spread pulse edges over the frame, including across the two boards.
  const uint16_t on = uint16_t((legIndex * JOINT_COUNT + jointIndex) * 4096 / SERVO_COUNT);
  const uint16_t off = uint16_t((on + ticks) % 4096);
  if (pwm[board].setPWM(channel, on, off) != 0)
  {
    console.printf("PCA9685 write failed: address 0x%02X, channel %u\n",
                  DRIVER_ADDRESSES[board], channel);
    return servo_error("I2C servo write failed.");
  }
  lastPulses[legIndex][jointIndex] = pulseUs;
  lastServoWriteMs = millis();
  return true;
}

bool queue_pulse(size_t legIndex, size_t jointIndex, uint16_t pulseUs)
{
  if (!servosReady)
  {
    console.println("ERROR: servo drivers not ready; use init.");
    return false;
  }
  targetPulses[legIndex][jointIndex] = pulseUs;
  return true;
}

bool queue_servo(size_t legIndex, size_t jointIndex, float angle)
{
  uint16_t pulseUs = 0;
  if (!servoPulse(angle, settings.servos[legIndex][jointIndex], pulseUs))
    return servo_error("Invalid servo angle or calibration.");
  return queue_pulse(legIndex, jointIndex, pulseUs);
}

void update_servo_outputs(uint32_t now)
{
  if (mode == Mode::Off || mode == Mode::ServoTest || mode == Mode::ServoTestPaused ||
      uint32_t(now - lastServoWriteMs) < servoWriteWaitMs)
    return;

  const size_t firstServo = nextServo;
  for (size_t offset = 0; offset < SERVO_COUNT; ++offset)
  {
    const size_t index = (firstServo + offset) % SERVO_COUNT;
    const size_t leg = index / JOINT_COUNT;
    const size_t joint = index % JOINT_COUNT;
    const uint16_t target = targetPulses[leg][joint];
    if (target == 0 || target == lastPulses[leg][joint])
      continue;
    const bool stagger = posePending || mode == Mode::Pose || lastPulses[leg][joint] == 0;
    if (!write_pulse(leg, joint, target))
      return;
    digitalWrite(SERVO_OE, LOW);
    servoWriteWaitMs = stagger ? SERVO_STAGGER_MS : 0;
    nextServo = (index + 1) % SERVO_COUNT;
    // Startup/poses are staggered; an active gait frame must finish before the next target.
    if (stagger)
      return;
  }
  if (posePending)
  {
    posePending = false;
    lastUpdateMs = now;
    if (mode == Mode::Walk)
      manualMotion.startedMs = now;
    console.println("Staggered pose ready.");
  }
}

bool clear_servo_outputs()
{
  for (size_t board = 0; board < DRIVER_COUNT; ++board)
    for (uint8_t channel = 0; channel < 16; ++channel)
      if (pwm[board].setPWM(channel, 0, 4096) != 0)
        return servo_error("Cannot clear PCA9685 outputs.");
  memset(lastPulses, 0, sizeof(lastPulses));
  memset(targetPulses, 0, sizeof(targetPulses));
  nextServo = 0;
  return true;
}

bool init_servos()
{
  disable_outputs();
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
      console.printf("Missing PCA9685 at 0x%02X\n", DRIVER_ADDRESSES[board]);
      return servo_error("Check PCA9685 power, wiring and address jumpers.");
    }
    for (uint8_t channel = 0; channel < 16; ++channel)
      if (pwm[board].setPWM(channel, 0, 4096) != 0)
        return servo_error("Cannot clear PCA9685 outputs.");
    pwm[board].setOscillatorFrequency(settings.oscillatorHz(board));
    pwm[board].setPWMFreq(settings.servoHz);
    pwmPrescale[board] = pwm[board].readPrescale();
    const uint8_t expected = uint8_t(lroundf(
        settings.oscillatorHz(board) / (4096.0f * settings.servoHz)) - 1);
    if (pwmPrescale[board] != expected)
      return servo_error("PCA9685 frequency configuration failed.");
  }
  servosReady = true;
  console.println("PCA9685 drivers ready; outputs remain disabled.");
  return true;
}

bool load_settings(Settings& loaded)
{
  if (!storageReady)
  {
    console.println("ERROR: flash settings storage unavailable.");
    return false;
  }
  if (!preferences.isKey("settings"))
  {
    console.println("No saved settings.");
    return false;
  }
  SettingsRecord record;
  if (preferences.getBytesLength("settings") != sizeof(record) ||
      preferences.getBytes("settings", &record, sizeof(record)) != sizeof(record))
  {
    console.println("ERROR: saved settings size/read failure; RAM settings unchanged.");
    return false;
  }
  const char* error = nullptr;
  if (!decodeSettings(&record, sizeof(record), loaded, error))
  {
    console.printf("ERROR: %s; RAM settings unchanged.\n", error);
    return false;
  }
  console.println("Settings loaded from flash; outputs remain disabled.");
  return true;
}

bool save_settings()
{
  if (!storageReady)
  {
    console.println("ERROR: flash settings storage unavailable.");
    return false;
  }
  const char* error = nullptr;
  if (!validateSettings(settings, error))
  {
    console.printf("ERROR: cannot save invalid settings: %s\n", error);
    return false;
  }
  const SettingsRecord record = makeSettingsRecord(settings);
  SettingsRecord readBack;
  if (preferences.putBytes("settings", &record, sizeof(record)) != sizeof(record) ||
      preferences.getBytes("settings", &readBack, sizeof(readBack)) != sizeof(readBack) ||
      memcmp(&record, &readBack, sizeof(record)) != 0)
  {
    console.println("ERROR: settings save/read-back verification failed.");
    return false;
  }
  settingsDirty = false;
  console.println("Settings saved to flash and verified.");
  return true;
}

void apply_settings(const Settings& candidate)
{
  if (settings.oscillator0Hz != candidate.oscillator0Hz ||
      settings.oscillator1Hz != candidate.oscillator1Hz ||
      settings.servoHz != candidate.servoHz || settings.i2cHz != candidate.i2cHz)
    servosReady = false;
  if (settings.radioChannel != candidate.radioChannel)
  {
    radioReady = false;
    wifiReady = false;
  }
  settings = candidate;
  settingsDirty = true;
  control = ControlState{};
  rebuild_geometry();
  if (!servosReady || !radioReady)
    console.println("Hardware configuration pending: use init before enabling affected outputs/radio.");
}

bool prepare_pose()
{
  if (mode != Mode::Off && mode != Mode::Pose)
    return require_off();
  if (!servosReady)
  {
    console.println("ERROR: servo drivers not ready; use init.");
    return false;
  }
  return mode == Mode::Pose || clear_servo_outputs();
}

void enable_pose()
{
  mode = Mode::Pose;
  posePending = true;
  console.println("Pose queued: one joint every 250 ms; ESP-NOW ignored."
                  " Other previously tested joints keep holding. stop cancels pending moves.");
}

void print_servo_test_step()
{
  const size_t leg = servoTest.servoIndex / JOINT_COUNT;
  const size_t joint = servoTest.servoIndex % JOINT_COUNT;
  console.printf("Servo test %u/%u: %s %s, address 0x%02X channel %u, nominal %.0f deg (%u us).\n",
      unsigned(servoTest.servoIndex + 1), unsigned(LEG_COUNT * JOINT_COUNT),
      LEG_NAMES[leg], JOINT_NAMES[joint], DRIVER_ADDRESSES[servoBoard(leg)],
      servoChannel(leg, joint), SERVO_TEST_ANGLES[servoTest.step], lastPulses[leg][joint]);
}

void print_servo_test_pause()
{
  const size_t leg = servoTest.servoIndex / JOINT_COUNT;
  console.printf("Leg %s test complete; outputs disabled. Paused: retest repeats this leg; next or Enter %s.\n",
      LEG_NAMES[leg], leg + 1 < LEG_COUNT ? "tests the next leg" : "finishes the sequence");
}

bool apply_servo_test_step()
{
  const size_t leg = servoTest.servoIndex / JOINT_COUNT;
  const size_t joint = servoTest.servoIndex % JOINT_COUNT;
  if (!queue_servo(leg, joint, SERVO_TEST_ANGLES[servoTest.step]) ||
      !write_pulse(leg, joint, targetPulses[leg][joint]))
    return false;
  servoWriteWaitMs = SERVO_STAGGER_MS;
  mode = Mode::ServoTest;
  digitalWrite(SERVO_OE, LOW);
  // Start a fresh hold after each write; delayed loops must not skip angles.
  servoTest.startedMs = millis();
  return true;
}

bool start_servo_test()
{
  if (!require_off() || !prepare_pose())
    return false;
  servoTest = ServoTestState{};
  if (!apply_servo_test_step())
    return false;
  console.println("Servo test started: one joint at a time, 90/85/95/90 deg, 1000 ms each."
                 " Pauses after each leg: retest repeats it, next or Enter advances."
                 " Support the robot; stop holds the current joint, !/off disables outputs.");
  print_servo_test_step();
  return true;
}

bool resume_servo_test(bool retest)
{
  if (mode != Mode::ServoTestPaused)
  {
    console.println("ERROR: next/retest requires testservos paused after a leg.");
    return false;
  }
  if (retest)
    servoTest.servoIndex = (servoTest.servoIndex / JOINT_COUNT) * JOINT_COUNT;
  else if (++servoTest.servoIndex == LEG_COUNT * JOINT_COUNT)
  {
    disable_outputs();
    console.println("Servo test complete; outputs disabled. Radio remains ignored.");
    return true;
  }
  servoTest.step = 0;
  if (!apply_servo_test_step())
    return false;
  print_servo_test_step();
  return true;
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
    targetPulses[leg][joint] = 0;
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
    console.println("ERROR: servo drivers not ready; use init.");
    return false;
  }
  if (mode == Mode::Off || mode == Mode::Pose || mode == Mode::ServoTest ||
      mode == Mode::ServoTestPaused)
  {
    if (mode == Mode::Off && !clear_servo_outputs())
      return false;
    rebuild_geometry();
    if (!set_foot_positions(0, 0))
      return false;
    posePending = true;
    console.println("Home queued: one joint every 250 ms; driving waits until ready."
                    " Support the body. stop cancels pending moves.");
  }
  control = ControlState{};
  webMotion = WebMotion{};
  mode = Mode::Home;
  return true;
}

void print_status()
{
  console.printf("mode=%s OE=%s drivers=%s radio=%s unsaved=%s MAC=%s\n",
      MODE_NAMES[size_t(mode)], digitalRead(SERVO_OE) == HIGH ? "HIGH" : "LOW",
      servosReady ? "ready" : "init-needed", radioReady ? "ready" : "init-needed",
      settingsDirty ? "yes" : "no", WiFi.macAddress().c_str());
  console.printf("pose-pending=%s (commanded targets, not position feedback)\n", posePending ? "yes" : "no");
  console.printf("armed=%u direction=%.2f rotation=%.2f configured-channel=%u AP=%s\n", control.armed,
      mode == Mode::Web ? webMotion.direction : (mode == Mode::Walk ? manualMotion.direction : control.direction),
      mode == Mode::Web ? webMotion.rotation : (mode == Mode::Walk ? manualMotion.rotation : control.rotation),
      unsigned(settings.radioChannel), apActive ? apName : "off");
  if (havePacket)
  {
    console.printf("last valid RX age=%lu ms CSV=", static_cast<unsigned long>(millis() - lastPacket.receivedAtMs));
    for (size_t i = 0; i < CONTROL_CHANNEL_COUNT; ++i)
      console.printf(i + 1 == CONTROL_CHANNEL_COUNT ? "%u\n" : "%u,", lastPacket.channels[i]);
  }
  else
    console.println("No valid RX packet since initialization.");
  if (mode == Mode::ServoTest)
    print_servo_test_step();
  else if (mode == Mode::ServoTestPaused)
    print_servo_test_pause();
  if (mode == Mode::Off || mode == Mode::Pose || mode == Mode::ServoTest ||
      mode == Mode::ServoTestPaused)
    for (size_t leg = 0; leg < LEG_COUNT; ++leg)
      console.printf("%s commanded us (0=off): %u %u %u\n", LEG_NAMES[leg],
                    lastPulses[leg][0], lastPulses[leg][1], lastPulses[leg][2]);
}

void print_help()
{
  console.println(
      "Commands (newline or CRLF, 115200 baud):\n"
      "  help | status | params | show\n"
      "  off | ! (immediate disable; finish with newline) | stop (return home if walking)\n"
      "  center [all|LEG] | servo LEG JOINT ANGLE | pulse LEG JOINT US\n"
      "  testservos (while off; one joint at a time, 90/85/95/90 deg, 1000 ms each)\n"
      "  next (or Enter on an empty line) | retest (only at the pause after each leg; outputs disabled while paused)\n"
      "  joints LEG COXA FEMUR TIBIA | foot LEG DX DY DZ (offset mm from home)\n"
      "  home | radio | walk DIRECTION ROTATION DURATION_MS (100..10000)\n"
      "  web (enable browser driving while fallback AP is available)\n"
      "  set NAME VALUE | cal LEG JOINT MIN_US MAX_US TRIM REVERSE\n"
      "  origin LEG X Y | save | load | defaults | init | scan\n"
      "LEG=rf/rm/rr/lr/lm/lf or 0..5; JOINT=coxa/femur/tibia or 0..2.\n"
      "Use off before configuration, storage, scan, init, help, params or show.\n"
      "Direct joint tests require off/pose. walk/home/radio explicitly take control.\n"
      "Open fallback AP: http://192.168.4.1. Fresh ESP-NOW disables AP and takes over web driving.\n"
      "next or Enter advances/finishes testservos; retest repeats the paused leg.\n"
      "stop cancels testservos holding the active joint or staying off if paused.\n"
      "Poses/startup stagger joints by 250 ms; stop cancels pending moves; driving waits for home.\n"
      "Only save writes settings. Boot always stays off; !/off releases torque.");
}

void show_settings(bool ranges)
{
  for (const FloatParameter& parameter : FLOAT_PARAMETERS)
  {
    if (ranges)
      console.printf("%s: %.9g..%.9g\n", parameter.name, parameter.minimum, parameter.maximum);
    else
      console.printf("set %s %.9g\n", parameter.name, settings.*(parameter.member));
  }
  for (const IntegerParameter& parameter : INTEGER_PARAMETERS)
  {
    if (ranges)
      console.printf("%s: %lu..%lu (integer)\n", parameter.name,
                    static_cast<unsigned long>(parameter.minimum), static_cast<unsigned long>(parameter.maximum));
    else
      console.printf("set %s %lu\n", parameter.name,
                    static_cast<unsigned long>(settings.*(parameter.member)));
  }
  if (ranges)
    return;
  for (size_t leg = 0; leg < LEG_COUNT; ++leg)
  {
    console.printf("origin %s %.9g %.9g\n", LEG_NAMES[leg], settings.origins[leg].x, settings.origins[leg].y);
    for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
    {
      const ServoCalibration& calibration = settings.servos[leg][joint];
      console.printf("cal %s %s %u %u %.9g %u\n", LEG_NAMES[leg], JOINT_NAMES[joint],
          calibration.minUs, calibration.maxUs, calibration.trimDegrees, calibration.reversed);
    }
  }
}

bool radio_active(uint32_t now)
{
  return radioReady && havePacket &&
      uint32_t(now - lastPacket.receivedAtMs) < settings.timeoutMs;
}

bool execute_command(const Command& command)
{
  if (command.type != CommandType::Status && command.type != CommandType::Help &&
      command.type != CommandType::Show && command.type != CommandType::Params)
    ++webGeneration;
  if (command.type == CommandType::Off)
  {
    disable_outputs();
    console.println("Outputs disabled.");
    return true;
  }
  if (command.type == CommandType::Status)
  {
    print_status();
    return true;
  }
  if (command.type == CommandType::Stop)
  {
    if (posePending)
    {
      memcpy(targetPulses, lastPulses, sizeof(targetPulses));
      posePending = false;
      mode = Mode::Pose;
    }
    else if (mode == Mode::Radio || mode == Mode::Walk || mode == Mode::Web)
      mode = Mode::Home;
    else if (mode == Mode::ServoTest)
    {
      mode = Mode::Pose;
      servoTest = ServoTestState{};
    }
    else if (mode == Mode::ServoTestPaused)
    {
      disable_outputs();
      console.println("Servo test cancelled; outputs remain disabled.");
      return true;
    }
    control = ControlState{};
    webMotion = WebMotion{};
    console.println("Motion stopped; current pose/home held unless outputs were already off.");
    return true;
  }
  if (command.type == CommandType::TestServos)
  {
    return start_servo_test();
  }
  if (command.type == CommandType::Next || command.type == CommandType::Retest)
  {
    return resume_servo_test(command.type == CommandType::Retest);
  }
  if (command.type == CommandType::Home || command.type == CommandType::Radio ||
      command.type == CommandType::Walk || command.type == CommandType::Web)
  {
    if (command.type == CommandType::Radio && !radioReady)
    {
      console.println("ERROR: radio not ready; use off then init.");
      return false;
    }
    if (command.type == CommandType::Web && (!apActive || !wifiReady || radio_active(millis())))
    {
      console.println("ERROR: web control requires the fallback AP and no active ESP-NOW signal.");
      return false;
    }
    if (!enter_home())
      return false;
    if (command.type == CommandType::Radio)
    {
      xQueueReset(controlQueue);
      mode = Mode::Radio;
      console.println("ESP-NOW mode: center sticks to arm. Serial tests do not resume radio automatically.");
    }
    else if (command.type == CommandType::Walk)
    {
      manualMotion = {command.values[0], command.values[1], millis(), command.durationMs};
      mode = Mode::Walk;
      console.println("Timed walking test queued; duration starts when home is ready."
                      " stop cancels startup/returns home, ! disables outputs.");
    }
    else if (command.type == CommandType::Web)
    {
      mode = Mode::Web;
      console.println("Web driving selected; waits for home startup before moving."
                      " No web timeout: link loss may leave motion running. Use stop/off to stop.");
    }
    else
      console.println("Home stance selected; radio ignored.");
    return true;
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
        console.println("ERROR: pulse outside this servo's calibrated min/max; use cal while off.");
        return false;
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
        console.println("ERROR: foot target outside leg reach.");
        return false;
      }
      calculate_ik(testLeg, angles);
      angles = Vector(90 + angles.x, 90 + angles.y, fabsf(angles.z));
      if (!isfinite(angles.x) || !isfinite(angles.y) || !isfinite(angles.z) ||
          angles.x < settings.coxaMin || angles.x > settings.coxaMax ||
          angles.y < 0 || angles.y > 180 || angles.z < 0 || angles.z > 180)
      {
        console.println("ERROR: foot target outside joint angle limits.");
        return false;
      }
    }
    if (!prepare_pose())
      return false;
    if (command.type == CommandType::Center)
    {
      for (size_t leg = 0; leg < LEG_COUNT; ++leg)
        if (command.leg == LEG_COUNT || command.leg == leg)
          for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
            if (!queue_servo(leg, joint, 90))
              return false;
    }
    else if (command.type == CommandType::Servo)
    {
      if (!queue_servo(command.leg, command.joint, command.values[0]))
        return false;
    }
    else if (command.type == CommandType::Pulse)
    {
      if (!queue_pulse(command.leg, command.joint, uint16_t(command.values[0])))
        return false;
    }
    else
    {
      const float values[] = {angles.x, angles.y, angles.z};
      for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
        if (!queue_servo(command.leg, joint,
                         command.type == CommandType::Foot ? values[joint] : command.values[joint]))
          return false;
    }
    enable_pose();
    return true;
  }

  if (!require_off())
    return false;
  switch (command.type)
  {
    case CommandType::Help: print_help(); break;
    case CommandType::Show: show_settings(false); break;
    case CommandType::Params: show_settings(true); break;
    case CommandType::Save: return save_settings();
    case CommandType::Init:
    {
      const bool servosOk = init_servos();
      const bool radioOk = init_control();
      return radioOk && servosOk;
    }
    case CommandType::Scan:
      if (!wireReady)
      {
        console.println("ERROR: I2C not initialized; use init.");
        return false;
      }
      else
      {
        unsigned found = 0;
        for (uint8_t address = 8; address < 120; ++address)
        {
          Wire.beginTransmission(address);
          const uint8_t result = Wire.endTransmission();
          if (result == 0)
          {
            console.printf("I2C device: 0x%02X\n", address);
            ++found;
          }
          else if (result != 2)
          {
            console.printf("ERROR: I2C scan failed at 0x%02X, code %u\n", address, result);
            return false;
          }
        }
        console.printf("Scan complete: %u responding address(es).\n", found);
      }
      break;
    case CommandType::Load:
    {
      Settings loaded;
      if (!load_settings(loaded))
        return false;
      apply_settings(loaded);
      settingsDirty = false;
      break;
    }
    case CommandType::Defaults:
      apply_settings(Settings{});
      console.println("Factory defaults in RAM only; save to replace saved settings.");
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
          console.printf("ERROR: %s\n", error);
          return false;
        }
      }
      else if (command.type == CommandType::Calibrate)
        candidate.servos[command.leg][command.joint] =
            {uint16_t(command.values[0]), uint16_t(command.values[1]), command.values[2], uint8_t(command.values[3])};
      else
        candidate.origins[command.leg] = {command.values[0], command.values[1]};
      if (!validateSettings(candidate, error))
      {
        console.printf("ERROR: %s\n", error);
        return false;
      }
      apply_settings(candidate);
      console.println("Setting updated in RAM; use save to persist.");
      break;
    }
    default: console.println("ERROR: unsupported command."); return false;
  }
  return true;
}

void read_console()
{
  for (size_t budget = 0; budget < 64 && Serial.available(); ++budget)
  {
    const LineEvent event = consoleLine.push(char(Serial.read()));
    if (event == LineEvent::Disable)
    {
      disable_outputs();
      console.println("Outputs disabled. Finish the line before the next command.");
      return;
    }
    if (event == LineEvent::Overflow || event == LineEvent::Invalid)
    {
      console.println("ERROR: invalid/overlong line discarded; finish with newline.");
      return;
    }
    if (event == LineEvent::Empty)
    {
      if (mode == Mode::ServoTestPaused)
        resume_servo_test(false);
      return;
    }
    if (event == LineEvent::Ready)
    {
      Command command;
      const char* error = nullptr;
      if (parseCommand(consoleLine.data, command, error))
        execute_command(command);
      else
        console.printf("ERROR: %s\n", error);
      return;
    }
  }
}

void update_network()
{
  const uint32_t now = millis();
  const bool wanted = wifiReady && !radio_active(now);
  if (wanted == apActive)
  {
    apRetryPending = false;
    return;
  }
  if (apRetryPending && uint32_t(now - lastApAttemptMs) < AP_RETRY_MS)
    return;
  lastApAttemptMs = now;
  apRetryPending = true;
  if (!wanted)
  {
    if (!WiFi.softAPdisconnect(true))
    {
      console.println("ERROR: cannot stop fallback AP; web commands remain blocked. Retrying.");
      return;
    }
    apActive = false;
    ++webGeneration;
    console.println("Fallback AP off; ESP-NOW has priority (or Wi-Fi needs init).");
  }
  else
  {
    if (!webTransport.begin())
    {
      console.println("ERROR: cannot start web server/queues; retrying.");
      return;
    }
    const auto mac = WiFi.macAddress();
    snprintf(apName, sizeof(apName), "Hexapod-%c%c%c%c%c%c",
        mac[9], mac[10], mac[12], mac[13], mac[15], mac[16]);
    if (!WiFi.mode(WIFI_AP_STA) ||
        !WiFi.softAP(apName, nullptr, int(settings.radioChannel), 0, 1))
    {
      console.println("ERROR: cannot start fallback AP; retrying.");
      if (!WiFi.softAPdisconnect(true))
        console.println("ERROR: cannot clean up failed AP startup.");
      return;
    }
    apActive = true;
    ++webGeneration;
    console.printf("Open AP %s: http://%s (no password; nearby devices can control the robot).\n",
        apName, WiFi.softAPIP().toString().c_str());
  }
  apRetryPending = false;
}

void write_web_status(WebResponse& response)
{
  snprintf(response.body, sizeof(response.body),
      "{\"mode\":\"%s\",\"generation\":%lu,\"token\":\"%s\",\"drivers\":%s,"
      "\"radioReady\":%s,\"radio\":false,\"dirty\":%s,\"leg\":\"%s\",\"joint\":\"%s\","
      "\"step\":%u,\"posePending\":%s,\"direction\":%.2f,\"rotation\":%.2f}",
      MODE_NAMES[size_t(mode)], static_cast<unsigned long>(webGeneration), webTransport.token,
      servosReady ? "true" : "false", radioReady ? "true" : "false",
      settingsDirty ? "true" : "false", LEG_NAMES[servoTest.servoIndex / JOINT_COUNT],
      JOINT_NAMES[servoTest.servoIndex % JOINT_COUNT], unsigned(servoTest.step),
      posePending ? "true" : "false",
      mode == Mode::Web ? webMotion.direction : control.direction,
      mode == Mode::Web ? webMotion.rotation : control.rotation);
}

void process_web_request(WebRequest& request, WebResponse& response)
{
  if (!apActive || !wifiReady || radio_active(millis()))
  {
    response.code = 409;
    console.println("ERROR: web unavailable; ESP-NOW takes precedence or Wi-Fi needs init.");
    return;
  }
  if (uint32_t(millis() - request.receivedMs) >= WEB_REQUEST_MAX_AGE_MS)
  {
    response.code = 409;
    console.println("ERROR: expired web request; retry explicitly.");
    return;
  }
  if (request.type == WebRequestType::Status)
  {
    write_web_status(response);
    return;
  }
  uint32_t generation = 0;
  char* payload = nullptr;
  if (!parseWebEnvelope(request.body, generation, payload))
  {
    response.code = 400;
    console.println("ERROR: malformed web command.");
    return;
  }
  Command command;
  if (request.type == WebRequestType::Command)
  {
    const char* error = nullptr;
    if (strcmp(payload, "!") == 0)
      command.type = CommandType::Off;
    else if (!parseCommand(payload, command, error))
    {
      response.code = 400;
      console.printf("ERROR: %s\n", error);
      return;
    }
  }
  const bool stopping = request.type == WebRequestType::Command &&
      (command.type == CommandType::Off || command.type == CommandType::Stop);
  if (generation != webGeneration && !stopping)
  {
    response.code = 409;
    console.println("ERROR: control source changed; refresh status and explicitly enable again.");
    return;
  }
  if (request.type == WebRequestType::Drive)
  {
    float direction = 0, rotation = 0;
    if (!parseWebMotion(payload, direction, rotation))
    {
      response.code = 400;
      console.println("ERROR: drive requires direction and rotation in -1..1.");
    }
    else if (mode != Mode::Web)
    {
      response.code = 409;
      console.println("ERROR: explicitly enable web driving first.");
    }
    else
    {
      webMotion = {direction, rotation};
      write_web_status(response);
    }
    return;
  }
  if (!execute_command(command))
    response.code = 400;
}

void read_web()
{
  WebRequest request;
  if (webTransport.requests == nullptr ||
      xQueueReceive(webTransport.requests, &request, 0) != pdTRUE)
    return;
  static WebResponse response;
  response = WebResponse{};
  response.id = request.id;
  console.capture(response.body, sizeof(response.body));
  process_web_request(request, response);
  if (console.truncated)
  {
    response.code = 500;
    const char suffix[] = "\nERROR: response truncated; use the serial console.\n";
    memcpy(response.body + sizeof(response.body) - sizeof(suffix), suffix, sizeof(suffix));
  }
  console.capture(nullptr, 0);
  response.generation = webGeneration;
  if (xQueueOverwrite(webTransport.responses, &response) != pdPASS)
    console.println("ERROR: cannot publish web response.");
}

void setup()
{
  // ESP32 requires GPIO registration before digitalWrite can preload the latch.
  pinMode(SERVO_OE, INPUT_PULLUP);
  digitalWrite(SERVO_OE, HIGH);
  pinMode(SERVO_OE, OUTPUT);
  disable_outputs();
  Serial.begin(115200);
  console.println("Hexapod serial console. Boot is always OFF; type help.");
  storageReady = preferences.begin("hexapod", false);
  if (!load_settings(settings))
    console.println("Using factory defaults in RAM; outputs remain disabled.");
  rebuild_geometry();
  init_servos();
  init_control();
  update_network();
  lastUpdateMs = millis();
}

void loop()
{
  read_control();
  update_network();
  if (mode == Mode::Walk && !posePending && manualMotion.expired(millis()))
  {
    mode = Mode::Home;
    console.println("Walking test complete; returning home. Radio remains ignored.");
  }
  read_console();
  read_web();
  const uint32_t now = millis();
  update_servo_test(now);
  if (!posePending && uint32_t(now - lastUpdateMs) >= settings.updateMs)
  {
    lastUpdateMs = now;
    if (mode == Mode::Web)
      set_foot_positions(webMotion.direction, webMotion.rotation);
    else if (mode == Mode::Radio)
      set_foot_positions(control.direction, control.rotation);
    else if (mode == Mode::Walk)
      set_foot_positions(manualMotion.direction, manualMotion.rotation);
    else if (mode == Mode::Home)
      set_foot_positions(0, 0);
  }
  update_servo_outputs(millis());
  delay(1);
}

void read_control()
{
  const uint32_t now = millis();
  if (mode == Mode::Radio && control.expire(now, settings.timeoutMs))
    console.println("ESP-NOW timeout: returning home; center sticks to re-arm.");

  ControlPacket packet;
  if (controlQueue != nullptr && xQueueReceive(controlQueue, &packet, 0) == pdTRUE)
  {
    if (!radioReady || uint32_t(now - packet.receivedAtMs) >= settings.timeoutMs)
      console.println("Discarded stale/unavailable ESP-NOW packet.");
    else
    {
      lastPacket = packet;
      havePacket = true;
      if (mode == Mode::Web)
      {
        webMotion = WebMotion{};
        control = ControlState{};
        mode = Mode::Radio;
        ++webGeneration;
        console.println("ESP-NOW takes precedence over web driving; center sticks to arm.");
      }
      const bool wasArmed = control.armed;
      if (mode == Mode::Radio)
      {
        if (!control.receive(packet, millis(), settings))
          console.println("Discarded stale ESP-NOW packet.");
        else if (!wasArmed && control.armed)
          console.println("ESP-NOW control armed.");
      }
    }
  }

  static uint32_t lastReportMs = 0;
  if (uint32_t(now - lastReportMs) >= 1000)
  {
    lastReportMs = now;
    const uint32_t rejected = rejectedPackets.exchange(0, std::memory_order_relaxed);
    if (rejected != 0)
      console.printf("Rejected %lu ESP-NOW packet(s); expected ten valid CSV fields.\n",
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

    if (!queue_servo(i, 0, ax) || !queue_servo(i, 1, ay) || !queue_servo(i, 2, az))
      return false;
  }
  return true;
}
