#include "../Hexapod_Arduino.ino"
#include <stdlib.h>

void check(bool condition, const char* expression, int line)
{
  if (!condition)
  {
    fprintf(stderr, "FAIL line %d: %s\nSerial:\n%s\n", line, expression, Serial.output.c_str());
    exit(1);
  }
}
#define CHECK(condition) check((condition), #condition, __LINE__)

void send(const std::string& text)
{
  Serial.output.clear();
  for (char character : text + "\n")
    Serial.input.push_back(character);
  while (!Serial.input.empty())
    loop();
}

void receive(const char* csv)
{
  on_control_received(nullptr, reinterpret_cast<const uint8_t*>(csv), int(strlen(csv)));
  loop();
}

void testBootAndPoses()
{
  setup();
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
  CHECK(servosReady && radioReady && Fake::saves == 0);
  for (auto& board : Fake::pwm)
    for (uint16_t ticks : board)
      CHECK(ticks == 4096);
  receive("0,2047,0,2047,4095,2047,1,1,1,1");
  CHECK(mode == Mode::Off && !control.armed);
  send("servo rf coxa 90");
  CHECK(mode == Mode::Pose && Fake::outputEnable == LOW);
  CHECK(Fake::pwm[0][0] == 307 && Fake::pwm[0][1] == 4096 && Fake::pwm[1][0] == 4096);
  send("pulse rf femur 1600");
  CHECK(lastPulses[0][0] == 1500 && lastPulses[0][1] == 1600);
  send("pulse rf femur 2300");
  CHECK(lastPulses[0][1] == 1600 && Serial.output.find("ERROR") != std::string::npos);
  send("set step_length 30");
  CHECK(settings.stepLength == 60 && Serial.output.find("off first") != std::string::npos);
  send("save");
  CHECK(Fake::saves == 0);
  send("off");
  send("servo lf tibia 90");
  CHECK(Fake::pwm[1][8] == 307 && Fake::pwm[0][0] == 4096);
  send("center all");
  for (auto& board : Fake::pwm)
    for (size_t channel = 0; channel < 16; ++channel)
      CHECK(board[channel] == (channel < 9 ? 307 : 4096));
  send("off");
  send("foot rf 0 0 0");
  CHECK(mode == Mode::Pose && lastPulses[0][0] == 1500 && lastPulses[0][2] > 0);
  send("joints lf 80 90 100");
  CHECK(lastPulses[5][0] < 1500 && lastPulses[5][2] > 1500);
  send("!");
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
  send(std::string(170, 'a') + "servo rf coxa 90");
  CHECK(mode == Mode::Off);
  send("status");
  CHECK(Serial.output.find("mode=off") != std::string::npos);
}

void testPersistence()
{
  send("set step_length 42");
  send("cal rf coxa 900 2100 5 1");
  send("origin rf 103 -60");
  send("set oscillator0_hz 25000001");
  CHECK(!servosReady && settings.oscillator0Hz == 25000001);
  send("servo rf coxa 90");
  CHECK(mode == Mode::Off && Serial.output.find("init") != std::string::npos);
  send("init");
  CHECK(servosReady && Fake::outputEnable == HIGH);
  send("save");
  CHECK(Fake::saves == 1 && !settingsDirty);
  const auto saved = Fake::saved;
  send("defaults");
  CHECK(settings.stepLength == 60 && settingsDirty && Fake::saved == saved);
  send("load");
  CHECK(settings.stepLength == 42 && settings.servos[0][0].minUs == 900);
  CHECK(settings.origins[0].x == 103 && !settingsDirty && Fake::outputEnable == HIGH);
  send("set step_length 40");
  Fake::saved[20] ^= 1;
  send("load");
  CHECK(settings.stepLength == 40 && Serial.output.find("ERROR") != std::string::npos);
  Fake::saved = saved;
  Fake::storageFailure = true;
  send("save");
  CHECK(settingsDirty && Serial.output.find("failed") != std::string::npos);
  Fake::storageFailure = false;
  Fake::readFailure = true;
  send("save");
  CHECK(settingsDirty && Serial.output.find("failed") != std::string::npos);
  Fake::readFailure = false;
  Fake::saved = saved;

  settings = Settings{};
  settingsDirty = false;
  setup(); // Simulated reboot with the NVS bytes preserved.
  CHECK(settings.stepLength == 42 && mode == Mode::Off && Fake::outputEnable == HIGH);
  CHECK(settings.servos[0][0].reversed == 1);
  send("defaults");
  send("init");
}

void testMotionAndRecovery()
{
  send("walk 1 0 10001");
  CHECK(mode == Mode::Off);
  send("walk 1 -0.5 10000");
  CHECK(mode == Mode::Walk && Fake::outputEnable == LOW);
  receive("0,2047,0,2047,0,2047,1,1,1,1");
  CHECK(mode == Mode::Walk && !control.armed);
  Fake::now = manualMotion.startedMs + 9999;
  loop();
  CHECK(mode == Mode::Walk);
  loop();
  CHECK(mode == Mode::Home && Fake::outputEnable == LOW);
  send("radio");
  CHECK(mode == Mode::Radio && !control.armed);
  receive("0,2047,0,2047,4095,2047,1,1,1,1");
  CHECK(!control.armed);
  receive("0,2047,2047,2047,2047,2047,1,1,1,1");
  CHECK(control.armed);
  receive("0,2047,0,2047,4095,2047,1,1,1,1");
  CHECK(control.direction == 1 && control.rotation == -1);
  Fake::now = control.lastPacketMs + 500;
  loop();
  CHECK(!control.armed && control.direction == 0);
  send("stop");
  receive("0,2047,2047,2047,2047,2047,1,1,1,1");
  CHECK(mode == Mode::Home && !control.armed);
  send("off");
  send("set radio_channel 6");
  CHECK(!radioReady);
  send("radio");
  CHECK(mode == Mode::Off);
  send("init");
  CHECK(Fake::channel == 6 && radioReady);
  send("home");
  CHECK(mode == Mode::Home);
  Fake::writeFailure = true;
  Fake::now += settings.updateMs;
  loop();
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !servosReady);
  send("status");
  CHECK(Serial.output.find("init-needed") != std::string::npos);
  Fake::writeFailure = false;
  Fake::boardsPresent = false;
  send("init");
  CHECK(!servosReady && mode == Mode::Off);
  send("help");
  CHECK(Serial.output.find("Commands") != std::string::npos);
  Fake::boardsPresent = true;
  send("init");
  CHECK(servosReady && mode == Mode::Off && Fake::outputEnable == HIGH);
}

int main()
{
  testBootAndPoses();
  testPersistence();
  testMotionAndRecovery();
  puts("PASS: actual firmware console, disabled boot, poses, NVS failures/reboot, timed motion and fault recovery.");
}
