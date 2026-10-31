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

void send(const std::string& text, const std::string& ending = "\n")
{
  Serial.output.clear();
  for (char character : text + ending)
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
  CHECK(Fake::pwm[0][7] == 307 && Fake::pwm[0][8] == 4096 &&
        Fake::pwm[0][0] == 4096 && Fake::pwm[1][0] == 4096);
  send("pulse rf femur 1600");
  CHECK(lastPulses[0][0] == 1500 && lastPulses[0][1] == 1600);
  CHECK(Fake::pwm[0][8] == 328 && Fake::pwm[0][7] == 307 && Fake::pwm[0][1] == 4096);
  send("pulse rf femur 2300");
  CHECK(lastPulses[0][1] == 1600 && Serial.output.find("ERROR") != std::string::npos);
  send("set step_length 30");
  CHECK(settings.stepLength == 60 && Serial.output.find("off first") != std::string::npos);
  send("save");
  CHECK(Fake::saves == 0);
  send("off");
  send("servo rr tibia 90");
  CHECK(Fake::pwm[0][15] == 307 && Fake::pwm[0][7] == 4096 && Fake::pwm[1][0] == 4096);
  send("off");
  send("servo lr coxa 90");
  CHECK(Fake::pwm[1][0] == 307 && Fake::pwm[1][7] == 4096 && Fake::pwm[0][7] == 4096);
  send("off");
  send("servo lf tibia 90");
  CHECK(Fake::pwm[1][8] == 307 && Fake::pwm[1][0] == 4096 &&
        Fake::pwm[1][15] == 4096 && Fake::pwm[0][7] == 4096);
  send("center all");
  for (size_t board = 0; board < DRIVER_COUNT; ++board)
    for (size_t channel = 0; channel < 16; ++channel)
      CHECK(Fake::pwm[board][channel] == ((board == 0 ? channel >= 7 : channel < 9) ? 307 : 4096));
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

void checkServoTestOutput(size_t servoIndex, uint16_t pulseUs)
{
  CHECK(mode == Mode::ServoTest && Fake::outputEnable == LOW);
  const size_t leg = servoIndex / JOINT_COUNT;
  const size_t joint = servoIndex % JOINT_COUNT;
  CHECK(servoTest.servoIndex == servoIndex);
  for (size_t i = 0; i < LEG_COUNT; ++i)
    for (size_t j = 0; j < JOINT_COUNT; ++j)
      CHECK(lastPulses[i][j] == (i == leg && j == joint ? pulseUs : 0));
  for (size_t board = 0; board < DRIVER_COUNT; ++board)
    for (size_t channel = 0; channel < 16; ++channel)
      CHECK(Fake::pwm[board][channel] ==
          (board == servoBoard(leg) && channel == servoChannel(leg, joint) ?
           pulseToTicks(pulseUs, settings.oscillatorHz(board), pwmPrescale[board]) : 4096));
}

void advanceServoTest()
{
  Fake::now = servoTest.startedMs + 1000;
  loop();
}

void checkServoTestPaused(size_t leg)
{
  CHECK(mode == Mode::ServoTestPaused && Fake::outputEnable == HIGH);
  CHECK(servoTest.servoIndex == (leg + 1) * JOINT_COUNT - 1);
  CHECK(servoTest.step == 3);
  for (auto& board : Fake::pwm)
    for (uint16_t ticks : board)
      CHECK(ticks == 4096);
  for (auto& pulses : lastPulses)
    for (uint16_t pulse : pulses)
      CHECK(pulse == 0);
}

void runServoTestLeg(size_t leg)
{
  const uint32_t started = servoTest.startedMs;
  const uint16_t pulses[] = {1500, 1472, 1528, 1500};
  for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
    for (size_t step = 0; step < 4; ++step)
    {
      const size_t servo = leg * JOINT_COUNT + joint;
      CHECK(servoTest.step == step);
      checkServoTestOutput(servo, pulses[step]);
      Fake::now = servoTest.startedMs + 999;
      loop();
      CHECK(servoTest.step == step);
      checkServoTestOutput(servo, pulses[step]);
      loop();
    }
  checkServoTestPaused(leg);
  CHECK(uint32_t(Fake::now - started) == 12001);
  CHECK(Serial.output.find(std::string("Leg ") + LEG_NAMES[leg] + " test complete") != std::string::npos);
}

void testServoSequence()
{
  send("testservos");
  const unsigned saves = Fake::saves;
  send("status");
  CHECK(Serial.output.find("mode=testservos") != std::string::npos);
  CHECK(Serial.output.find("rf coxa, address 0x40 channel 7") != std::string::npos);
  receive("0,2047,0,2047,4095,2047,1,1,1,1");
  CHECK(!control.armed && control.direction == 0 && control.rotation == 0);
  for (size_t leg = 0; leg < LEG_COUNT; ++leg)
    for (unsigned attempt = 0; attempt < 3; ++attempt)
    {
      runServoTestLeg(leg);
      if (leg + 1 == LEG_COUNT)
      {
        CHECK(Serial.output.find("lf tibia, address 0x41 channel 8") != std::string::npos);
        CHECK(Serial.output.find("next or Enter finishes the sequence") != std::string::npos);
      }
      Fake::now += 5000;
      loop();
      checkServoTestPaused(leg);
      send("status");
      CHECK(Serial.output.find("mode=testservos-paused OE=HIGH") != std::string::npos);
      CHECK(Serial.output.find(std::string("Leg ") + LEG_NAMES[leg] + " test complete") != std::string::npos);
      receive("0,2047,0,2047,4095,2047,1,1,1,1");
      CHECK(!control.armed && control.direction == 0 && control.rotation == 0);
      checkServoTestPaused(leg);
      send(attempt < 2 ? "retest" : "next");
    }
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !control.armed);
  CHECK(Fake::saves == saves);
  CHECK(Serial.output.find("Servo test complete; outputs disabled") != std::string::npos);
  for (auto& board : Fake::pwm)
    for (uint16_t ticks : board)
      CHECK(ticks == 4096);
  for (auto& leg : lastPulses)
    for (uint16_t pulse : leg)
      CHECK(pulse == 0);
  receive("0,2047,2047,2047,2047,2047,1,1,1,1");
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
}

void testServoEnterToAdvance()
{
  for (const char* ending : {"\n", "\r", "\r\n"})
  {
    send("", ending);
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && Serial.output.empty());
    send("testservos", ending);
    for (size_t leg = 0; leg < LEG_COUNT; ++leg)
    {
      send("", ending);
      checkServoTestOutput(leg * JOINT_COUNT, 1500);
      CHECK(servoTest.step == 0 && Serial.output.empty());
      runServoTestLeg(leg);
      send("status", ending);
      checkServoTestPaused(leg);
      CHECK(Serial.output.find("ERROR") == std::string::npos);
      if (leg == 0)
      {
        const std::string invalid[] = {" ", std::string(160, 'a'), std::string(1, '\0')};
        for (const std::string& text : invalid)
        {
          send(text, ending);
          checkServoTestPaused(leg);
          CHECK(Serial.output.find("ERROR") != std::string::npos);
        }
        send("status", "\r");
        checkServoTestPaused(leg);
        send("");
        checkServoTestPaused(leg);
        CHECK(Serial.output.empty());
      }
      send("", ending);
      CHECK(Serial.output.find("ERROR") == std::string::npos);
      if (leg + 1 < LEG_COUNT)
      {
        checkServoTestOutput((leg + 1) * JOINT_COUNT, 1500);
        CHECK(servoTest.step == 0);
      }
      else
      {
        CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !control.armed);
        CHECK(Serial.output.find("Servo test complete; outputs disabled") != std::string::npos);
      }
    }
  }
}

void testServoTestControls()
{
  for (const char* command : {"next", "retest"})
  {
    send(command);
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
    CHECK(Serial.output.find("requires testservos paused") != std::string::npos);
  }
  send("servo lf tibia 90");
  send("testservos");
  CHECK(mode == Mode::Pose && lastPulses[5][2] == 1500);
  CHECK(Serial.output.find("off first") != std::string::npos);
  send("off");
  send("testservos extra");
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
  send("testservos");
  checkServoTestOutput(0, 1500);
  const char* blocked[] = {"testservos", "servo lf tibia 90", "save", "init", "set step_length 30"};
  for (const char* command : blocked)
  {
    send(command);
    CHECK(Serial.output.find("off first") != std::string::npos);
    checkServoTestOutput(0, 1500);
  }
  for (const char* command : {"next", "retest"})
  {
    send(command);
    CHECK(Serial.output.find("requires testservos paused") != std::string::npos);
    checkServoTestOutput(0, 1500);
  }
  for (char character : std::string("servo rf"))
    Serial.input.push_back(character);
  advanceServoTest();
  checkServoTestOutput(0, 1472);
  send("");
  CHECK(Serial.output.find("ERROR") != std::string::npos);
  checkServoTestOutput(0, 1472);
  send("stop");
  CHECK(mode == Mode::Pose && Fake::outputEnable == LOW);
  CHECK(lastPulses[0][0] == 1472);
  Fake::now += 5000;
  loop();
  CHECK(mode == Mode::Pose && lastPulses[0][0] == 1472);
  send("off");

  for (const char* command : {"off", "!"})
  {
    send("testservos");
    advanceServoTest();
    send(command);
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
    Fake::now += 5000;
    loop();
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
  }

  const struct { const char* command; Mode expected; } handovers[] =
  {
    {"home", Mode::Home}, {"radio", Mode::Radio}, {"walk 0.25 0 100", Mode::Walk}
  };
  for (const auto& handover : handovers)
    for (bool paused : {false, true})
    {
      send("testservos");
      if (paused)
        runServoTestLeg(0);
      send(handover.command);
      CHECK(mode == handover.expected && Fake::outputEnable == LOW && !control.armed);
      for (auto& leg : lastPulses)
        for (uint16_t pulse : leg)
          CHECK(pulse > 0);
      send("testservos");
      CHECK(mode == handover.expected && Serial.output.find("off first") != std::string::npos);
      for (const char* command : {"next", "retest"})
      {
        send(command);
        CHECK(mode == handover.expected && Serial.output.find("requires testservos paused") != std::string::npos);
      }
      Fake::now += 5000;
      loop();
      CHECK(mode != Mode::ServoTest && mode != Mode::ServoTestPaused);
      send("off");
    }
}

void testServoPausedControls()
{
  send("testservos");
  runServoTestLeg(0);
  const char* blocked[] = {"testservos", "servo lf tibia 90", "save", "init", "set step_length 30"};
  for (const char* command : blocked)
  {
    send(command);
    CHECK(Serial.output.find("off first") != std::string::npos);
    checkServoTestPaused(0);
  }
  for (const char* command : {"next extra", "retest rf"})
  {
    send(command);
    CHECK(Serial.output.find("ERROR") != std::string::npos);
    checkServoTestPaused(0);
  }
  for (char character : std::string("ret"))
    Serial.input.push_back(character);
  Fake::now += 5000;
  loop();
  checkServoTestPaused(0);
  send("est");
  checkServoTestOutput(0, 1500);
  runServoTestLeg(0);
  send("off");

  for (const char* cancel : {"off", "!", "stop"})
  {
    send("testservos");
    runServoTestLeg(0);
    send(cancel);
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
    Fake::now += 5000;
    loop();
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
    for (const char* command : {"next", "retest"})
    {
      send(command);
      CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
      CHECK(Serial.output.find("requires testservos paused") != std::string::npos);
    }
  }
}

void testServoTestTimingAndFaults()
{
  send("cal rf coxa 900 2100 5 1");
  send("testservos");
  checkServoTestOutput(0, 1533);
  advanceServoTest();
  checkServoTestOutput(0, 1567);
  advanceServoTest();
  checkServoTestOutput(0, 1500);
  advanceServoTest();
  checkServoTestOutput(0, 1533);
  for (int step = 0; step < 9; ++step)
    advanceServoTest();
  checkServoTestPaused(0);
  send("retest");
  checkServoTestOutput(0, 1533);
  advanceServoTest();
  checkServoTestOutput(0, 1567);
  send("off");
  send("defaults");

  Fake::now = UINT32_MAX - 500;
  send("testservos");
  Fake::now = servoTest.startedMs + 999;
  loop();
  checkServoTestOutput(0, 1500);
  loop();
  checkServoTestOutput(0, 1472);
  CHECK(servoTest.startedMs < 1000);
  Fake::now = servoTest.startedMs + 15000;
  loop();
  CHECK(servoTest.step == 2);
  checkServoTestOutput(0, 1528);
  loop();
  checkServoTestOutput(0, 1528);
  send("off");

  for (int writes : {0, 32})
  {
    Fake::writesUntilFailure = writes;
    send("testservos");
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !servosReady);
    CHECK(Serial.output.find("ERROR") != std::string::npos);
    Fake::writesUntilFailure = -1;
    send("testservos");
    CHECK(mode == Mode::Off && Serial.output.find("init") != std::string::npos);
    send("init");
    CHECK(servosReady && mode == Mode::Off && Fake::outputEnable == HIGH);
  }

  for (int step : {0, 3})
  {
    send("testservos");
    for (int i = 0; i < step; ++i)
      advanceServoTest();
    Fake::writeFailure = true;
    advanceServoTest();
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !servosReady);
    CHECK(Serial.output.find("ERROR") != std::string::npos);
    Fake::writeFailure = false;
    send("init");
    CHECK(servosReady && mode == Mode::Off && Fake::outputEnable == HIGH);
  }

  send("testservos");
  for (int step = 0; step < 3; ++step)
    advanceServoTest();
  Fake::writesUntilFailure = 1;
  advanceServoTest();
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !servosReady);
  CHECK(Fake::pwm[0][7] == 4096 && Fake::pwm[0][8] == 4096);
  CHECK(Serial.output.find("ERROR") != std::string::npos);
  Fake::writesUntilFailure = -1;
  send("init");

  send("testservos");
  for (int step = 0; step < 11; ++step)
    advanceServoTest();
  Fake::writeFailure = true;
  advanceServoTest();
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !servosReady);
  CHECK(Serial.output.find("Cannot disable tested servo") != std::string::npos);
  Fake::writeFailure = false;
  send("init");

  for (const char* command : {"next", "retest"})
  {
    send("testservos");
    runServoTestLeg(0);
    Fake::writeFailure = true;
    send(command);
    CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !servosReady);
    CHECK(Serial.output.find("ERROR") != std::string::npos);
    for (auto& leg : lastPulses)
      for (uint16_t pulse : leg)
        CHECK(pulse == 0);
    Fake::writeFailure = false;
    send(command);
    CHECK(mode == Mode::Off && Serial.output.find("requires testservos paused") != std::string::npos);
    send("init");
    CHECK(servosReady && mode == Mode::Off && Fake::outputEnable == HIGH);
  }
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
  testServoSequence();
  testServoEnterToAdvance();
  testServoTestControls();
  testServoPausedControls();
  testServoTestTimingAndFaults();
  testPersistence();
  testMotionAndRecovery();
  puts("PASS: actual firmware console, disabled boot, poses, sequential servo tests, NVS failures/reboot, timed motion and fault recovery.");
}
