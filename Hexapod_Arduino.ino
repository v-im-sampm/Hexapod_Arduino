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
#include <Adafruit_PWMServoDriver.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include <math.h>
#include "HexapodControl.h"

#define POW2(x)       ((x)*(x))

using namespace Hexapod;

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr int SERVO_OE = 23;
constexpr uint8_t DRIVER_ADDRESSES[DRIVER_COUNT] = {0x40, 0x41};
constexpr uint32_t OSCILLATOR_HZ[DRIVER_COUNT] = {25000000, 25000000};
constexpr float SERVO_FREQUENCY_HZ = 50.0f;
constexpr uint32_t UPDATE_INTERVAL_MS = 10;
constexpr bool CENTER_SERVOS_ONLY = false;

// RF, RM, RR, LR, LM, LF; each row is coxa, femur, tibia.
const ServoCalibration SERVO_CALIBRATION[LEG_COUNT][JOINT_COUNT] =
{
  { {}, {}, {} },
  { {}, {}, {} },
  { {}, {}, {} },
  { {}, {}, {} },
  { {}, {}, {} },
  { {}, {}, {} }
};

Adafruit_PWMServoDriver pwm[DRIVER_COUNT] =
{
  Adafruit_PWMServoDriver(DRIVER_ADDRESSES[0]),
  Adafruit_PWMServoDriver(DRIVER_ADDRESSES[1])
};
uint8_t pwmPrescale[DRIVER_COUNT] = {};
QueueHandle_t controlQueue = nullptr;
std::atomic<uint32_t> rejectedPackets{0};
ControlState control;
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

  Leg(float _x, float _y) : origin(_x, _y, 0)
  {
  }
};

const float COXA_LENGTH  = 38.0;
const float FEMUR_LENGTH = 86.25;
const float TIBIA_LENGTH = 160.5;

const float STEP_LENGTH = 60.0;
const float STEP_HEIGHT = 100.0;
const float DEFAULT_Z = -40.0;

const float SMOOTHING_FACTOR = 0.4;
const float ROTATION_STEP_RAD = 15.0 * M_PI / 180.0;

Leg legs[LEG_COUNT] =
{
  Leg(100.94, -57.11),    // RF
  Leg(0.0, -105.72),      // RM
  Leg(-100.94, -57.11),   // RR
  Leg(-100.94, 57.11),    // LR
  Leg(0.0, 105.72),       // LM
  Leg(100.94, 57.11)      // LF
};

float t = 0;

void calculate_ik(const Leg& leg, Vector& res);
Vector get_foot_target(float t, size_t legIndex, float direction, float rotation);
void set_foot_positions(float direction, float rotation);
void read_control();

void halt_with_error(const char* message)
{
  digitalWrite(SERVO_OE, HIGH);
  Serial.printf("ERROR: %s Outputs disabled; fix the fault and reset.\n", message);
  while (true)
    delay(1000);
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

void init_control()
{
  controlQueue = xQueueCreate(1, sizeof(ControlPacket));
  if (controlQueue == nullptr)
    halt_with_error("Cannot allocate control queue.");
  if (!WiFi.mode(WIFI_STA))
    halt_with_error("Cannot start Wi-Fi station.");
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
      esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    halt_with_error("Cannot configure ESP-NOW channel/power saving.");
  Serial.printf("Receiver STA MAC: %s; ESP-NOW channel: %u\n",
                WiFi.macAddress().c_str(), ESPNOW_CHANNEL);
  if (esp_now_init() != ESP_OK ||
      esp_now_register_recv_cb(on_control_received) != ESP_OK)
    halt_with_error("Cannot initialize ESP-NOW receiver.");
}

void write_servo(size_t legIndex, size_t jointIndex, float angle)
{
  uint16_t pulseUs = 0;
  if (!servoPulse(angle, SERVO_CALIBRATION[legIndex][jointIndex], pulseUs))
    halt_with_error("Invalid servo angle or calibration.");
  const size_t board = servoBoard(legIndex);
  const uint8_t channel = servoChannel(legIndex, jointIndex);
  const uint16_t ticks = pulseToTicks(pulseUs, OSCILLATOR_HZ[board], pwmPrescale[board]);
  if (ticks == 0 || ticks >= 4096)
    halt_with_error("Servo pulse outside PCA9685 range.");
  if (pwm[board].setPWM(channel, 0, ticks) != 0)
  {
    Serial.printf("PCA9685 write failed: address 0x%02X, channel %u\n",
                  DRIVER_ADDRESSES[board], channel);
    halt_with_error("I2C servo write failed.");
  }
}

void init_servos()
{
  if (!Wire.begin(I2C_SDA, I2C_SCL))
    halt_with_error("Cannot initialize I2C.");
  Wire.setClock(400000);
  Wire.setTimeOut(20);

  for (size_t board = 0; board < DRIVER_COUNT; ++board)
  {
    if (!pwm[board].begin())
    {
      Serial.printf("Missing PCA9685 at 0x%02X\n", DRIVER_ADDRESSES[board]);
      halt_with_error("Check PCA9685 power, wiring and address jumpers.");
    }
    pwm[board].setOscillatorFrequency(OSCILLATOR_HZ[board]);
    pwm[board].setPWMFreq(SERVO_FREQUENCY_HZ);
    pwmPrescale[board] = pwm[board].readPrescale();
    const uint8_t expected = uint8_t(lroundf(
        OSCILLATOR_HZ[board] / (4096.0f * SERVO_FREQUENCY_HZ)) - 1);
    if (pwmPrescale[board] != expected)
      halt_with_error("PCA9685 frequency configuration failed.");
    for (uint8_t channel = 0; channel < 16; ++channel)
      if (pwm[board].setPWM(channel, 0, 4096) != 0)
        halt_with_error("Cannot clear PCA9685 outputs.");
  }

  for (size_t leg = 0; leg < LEG_COUNT; ++leg)
    for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
      write_servo(leg, joint, 90.0f);
  digitalWrite(SERVO_OE, LOW);
}

void setup()
{
  pinMode(SERVO_OE, OUTPUT);
  digitalWrite(SERVO_OE, HIGH);
  Serial.begin(115200);
  init_control();
  init_servos();

  const float homeReach = COXA_LENGTH + (FEMUR_LENGTH + TIBIA_LENGTH) * cos(M_PI / 4);
  for (size_t i = 0; i < LEG_COUNT; ++i)
  {
    legs[i].origin.z = atan2(legs[i].origin.y, legs[i].origin.x);
    legs[i].home = Vector(legs[i].origin.x + homeReach * cos(legs[i].origin.z),
                          legs[i].origin.y + homeReach * sin(legs[i].origin.z),
                          DEFAULT_Z);
    legs[i].target = legs[i].home;
  }

  Serial.println(CENTER_SERVOS_ONLY ? "Calibration: holding all joints at 90 degrees." :
                 "Ready: center both sticks to enable walking.");
  delay(1000);
  lastUpdateMs = millis();
}

void loop()
{
  read_control();
  const uint32_t now = millis();
  if (uint32_t(now - lastUpdateMs) < UPDATE_INTERVAL_MS)
  {
    delay(1);
    return;
  }
  lastUpdateMs = now;
  if (!CENTER_SERVOS_ONLY)
    set_foot_positions(control.direction, control.rotation);
}

void read_control()
{
  const uint32_t now = millis();
  if (control.expire(now))
    Serial.println("ESP-NOW timeout: returning home; center sticks to re-arm.");

  ControlPacket packet;
  if (xQueueReceive(controlQueue, &packet, 0) == pdTRUE)
  {
    const bool wasArmed = control.armed;
    if (!control.receive(packet, millis()))
      Serial.println("Discarded stale ESP-NOW packet.");
    else if (!wasArmed && control.armed)
      Serial.println("ESP-NOW control armed.");
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
  const float l_forward = l_xy - COXA_LENGTH;
  float D = sqrt(POW2(l_forward) + POW2(relativeTarget.z));

  if (D > (FEMUR_LENGTH + TIBIA_LENGTH)) 
  {
    D = FEMUR_LENGTH + TIBIA_LENGTH - 0.001;
  }
  if (D < fabs(FEMUR_LENGTH - TIBIA_LENGTH)) 
  {
    D = fabs(FEMUR_LENGTH - TIBIA_LENGTH) + 0.001;
  }

  const float kneeCos = constrain((POW2(FEMUR_LENGTH) + POW2(TIBIA_LENGTH) - POW2(D)) /
                                 (2.0 * FEMUR_LENGTH * TIBIA_LENGTH), -1.0, 1.0);
  res.z = (acos(kneeCos) - M_PI) * 180.0 / M_PI;

  const float beta1 = atan2(relativeTarget.z, l_forward);
  const float hipCos = constrain((POW2(FEMUR_LENGTH) + POW2(D) - POW2(TIBIA_LENGTH)) /
                                (2.0 * FEMUR_LENGTH * D), -1.0, 1.0);
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
    step_x_move = direction * (0.5 - phase_norm) * STEP_LENGTH;
    step_rot_rad = rotation * (0.5 - phase_norm) * ROTATION_STEP_RAD;
    step_z_move = 0;
  }
  else
  {
    float phase_norm = (phase - 0.5) * 2.0; // 0 -> 1
    step_x_move = direction * (phase_norm - 0.5) * STEP_LENGTH;
    step_rot_rad = rotation * (phase_norm - 0.5) * ROTATION_STEP_RAD;
    step_z_move = sin(phase_norm * M_PI) * STEP_HEIGHT;
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

void set_foot_positions(float direction, float rotation)
{
  t = fmod(t + 0.015, 1.0);

  Vector angles;
  for (size_t i = 0; i < LEG_COUNT; ++i)
  {
    const Vector goalTarget = get_foot_target(t, i, direction, rotation);

    legs[i].target.x += (goalTarget.x - legs[i].target.x) * SMOOTHING_FACTOR;
    legs[i].target.y += (goalTarget.y - legs[i].target.y) * SMOOTHING_FACTOR;
    legs[i].target.z += (goalTarget.z - legs[i].target.z) * SMOOTHING_FACTOR;

    calculate_ik(legs[i], angles);

    float ax = 90 + angles.x;
    float ay = 90 + angles.y;
    float az = fabs(angles.z);

    ax = constrain(ax, 45, 135);

    write_servo(i, 0, ax);
    write_servo(i, 1, ay);
    write_servo(i, 2, az);
  }
}
