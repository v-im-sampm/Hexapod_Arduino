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

void finishPose()
{
  for (unsigned elapsed = 0; posePending && elapsed < 10000; ++elapsed)
    loop();
  CHECK(!posePending);
}

void send(const std::string& text, const std::string& ending = "\n", bool waitForPose = true)
{
  Serial.output.clear();
  for (char character : text + ending)
    Serial.input.push_back(character);
  while (!Serial.input.empty())
    loop();
  if (waitForPose)
    finishPose();
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
  CHECK(Fake::unconfiguredGpioWrites == 0);
  CHECK(Fake::oeStartupLevels.size() == 1 && Fake::oeStartupLevels.front() == HIGH);
  CHECK(servosReady && radioReady && Fake::saves == 0);
  CHECK(apActive && Fake::apStarted && Fake::wifiMode == WIFI_AP_STA);
  for (int oe : Fake::driverOutputEnable)
    CHECK(oe == HIGH);
  for (int oe : Fake::radioOutputEnable)
    CHECK(oe == HIGH);
  for (unsigned active : Fake::radioActiveChannels)
    CHECK(active == 0);
  CHECK(Fake::apName == "Hexapod-334455" && Fake::channel == settings.radioChannel);
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
  for (int level : Fake::oeStartupLevels)
    CHECK(level == HIGH);
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
  send("walk 1 0 1000");
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

httpd_req_t http(const char* path, const std::string& body = "", bool authorized = true,
                bool serviceMainLoop = true)
{
  httpd_req_t request;
  request.uri = path;
  request.method = strcmp(path, "/") == 0 || strcmp(path, "/api/status") == 0 ? HTTP_GET : HTTP_POST;
  request.body = body;
  request.content_len = body.size();
  if (authorized)
    request.headers["X-Hexapod-Token"] = webTransport.token;
  Fake::waitHook = serviceMainLoop ? loop : nullptr;
  for (const auto& route : fakeHttpServer.routes)
    if (strcmp(route.uri, path) == 0 && route.method == request.method)
    {
      request.user_ctx = route.user_ctx;
      CHECK(route.handler(&request) == ESP_OK);
      return request;
    }
  CHECK(false);
  return request;
}

httpd_req_t webCommand(const std::string& command)
{
  return http("/api/command", std::to_string(webGeneration) + "\n" + command);
}

httpd_req_t webDrive(const std::string& axes, uint32_t generation)
{
  return http("/api/drive", std::to_string(generation) + "\n" + axes);
}

void finishWebPose()
{
  for (unsigned elapsed = 0; posePending && elapsed < 10000; ++elapsed)
  {
    if (elapsed % 50 == 0)
      CHECK(webDrive("0 0", webGeneration).status == "200 OK");
    loop();
  }
  CHECK(!posePending && mode == Mode::Web);
}

void prepareWebTest()
{
  send("off");
  send("defaults");
  send("init");
  loop();
  CHECK(apActive && Fake::apStarted && Fake::wifiMode == WIFI_AP_STA);
  CHECK(Fake::channel == settings.radioChannel);
}

void testWebCommands()
{
  prepareWebTest();
  const auto page = http("/");
  CHECK(page.status == "200 OK" && page.contentType == "text/html; charset=utf-8");
  CHECK(page.response.find("Hexapod control") != std::string::npos);
  CHECK(page.responseHeaders.at("Cache-Control") == "no-store");
  CHECK(page.responseHeaders.at("Content-Security-Policy").find("frame-ancestors 'none'") != std::string::npos);
  const auto status = http("/api/status");
  CHECK(status.status == "200 OK" && status.contentType == "application/json");
  CHECK(status.response.find("\"mode\":\"off\"") != std::string::npos);
  CHECK(status.response.find(webTransport.token) != std::string::npos);
  CHECK(status.responseHeaders.at("X-Hexapod-Generation") == std::to_string(webGeneration));
  CHECK(http("/api/command", "0\nweb", false).status == "403 Forbidden");
  CHECK(http("/api/command", std::string(192, 'a')).status == "413 Content Too Large");
  CHECK(http("/api/command", "").status == "413 Content Too Large");
  CHECK(http("/api/command", "web").status == "400 Bad Request");
  CHECK(http("/api/command", "4294967296\nweb").status == "400 Bad Request");
  CHECK(webCommand("web\nservo rf coxa 90").status == "400 Bad Request");
  CHECK(webCommand(std::string("we\0b", 4)).status == "400 Bad Request");
  CHECK(webCommand(std::string(160, 'x')).status == "400 Bad Request");
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);

  CHECK(webCommand("cal rf coxa 1000 2000 0 0").status == "200 OK");
  CHECK(webCommand("cal rf coxa 2000 1000 0 0").status == "400 Bad Request");
  CHECK(settings.servos[0][0].minUs == 1000 && settings.servos[0][0].maxUs == 2000);
  CHECK(webCommand("set step_length 40").status == "200 OK" && settings.stepLength == 40);
  CHECK(webCommand("set step_length nan").status == "400 Bad Request" && settings.stepLength == 40);
  CHECK(webCommand("show").response.find("set step_length 40") != std::string::npos);
  CHECK(webCommand("show").response.find("cal rf coxa 1000 2000 0 0") != std::string::npos);
  CHECK(webCommand("params").status == "200 OK");
  CHECK(webCommand("help").status == "200 OK");
  CHECK(webCommand("servo rf coxa 90").status == "200 OK" && mode == Mode::Pose);
  const auto blocked = webCommand("save");
  CHECK(blocked.status == "400 Bad Request" && blocked.response.find("off first") != std::string::npos);
  CHECK(webCommand("off").status == "200 OK");
  Fake::storageFailure = true;
  CHECK(webCommand("save").status == "400 Bad Request");
  Fake::storageFailure = false;
  CHECK(webCommand("defaults").status == "200 OK");
  CHECK(webCommand("testservos").status == "200 OK");
  runServoTestLeg(0);
  CHECK(http("/api/status").response.find("\"mode\":\"testservos-paused\"") != std::string::npos);
  CHECK(webCommand("retest").status == "200 OK");
  checkServoTestOutput(0, 1500);
  runServoTestLeg(0);
  CHECK(webCommand("next").status == "200 OK");
  checkServoTestOutput(3, 1500);
  CHECK(webCommand("!").status == "200 OK" && mode == Mode::Off);
}

void testWebDrivingAndTimeout()
{
  prepareWebTest();
  CHECK(webDrive("1 0", webGeneration).status == "409 Conflict");
  CHECK(webCommand("web").status == "200 OK");
  CHECK(posePending);
  const auto pendingDrive = webDrive("0 0", webGeneration);
  CHECK(pendingDrive.status == "200 OK" && pendingDrive.contentType == "application/json");
  CHECK(pendingDrive.response.find("\"mode\":\"web\"") != std::string::npos);
  CHECK(pendingDrive.response.find("\"posePending\":true") != std::string::npos);
  CHECK(pendingDrive.response.find("\"direction\":0.00,\"rotation\":0.00") != std::string::npos);
  CHECK(pendingDrive.response.find("\"generation\":" + std::to_string(webGeneration) + ",") != std::string::npos);
  CHECK(pendingDrive.response.find(webTransport.token) != std::string::npos);
  finishWebPose();
  CHECK(mode == Mode::Web && Fake::outputEnable == LOW);
  const uint32_t generation = webGeneration;
  const auto drive = webDrive("0.5 -0.5", generation);
  CHECK(drive.status == "200 OK" && drive.contentType == "application/json");
  CHECK(drive.response.find("\"posePending\":false") != std::string::npos);
  CHECK(drive.response.find("\"direction\":0.50,\"rotation\":-0.50") != std::string::npos);
  CHECK(webMotion.direction == 0.5f && webMotion.rotation == -0.5f);
  const uint32_t received = webMotion.receivedMs;
  for (const char* axes : {"nan 0", "0 inf", "2 0", "0 -2", "1", "1 0 extra", "1 "})
    CHECK(webDrive(axes, generation).status == "400 Bad Request");
  CHECK(webMotion.receivedMs == received);
  Fake::now = received + WEB_TIMEOUT_MS - 1;
  loop();
  CHECK(mode == Mode::Web);
  loop();
  CHECK(mode == Mode::Home && webMotion.direction == 0 && webMotion.rotation == 0);
  const auto expiredDrive = webDrive("1 0", generation);
  CHECK(expiredDrive.status == "409 Conflict");
  CHECK(expiredDrive.contentType == "text/plain; charset=utf-8");
  CHECK(mode == Mode::Home);
  CHECK(webCommand("web").status == "200 OK");
  const uint32_t restartedGeneration = webGeneration;
  CHECK(webDrive("1 0", generation).status == "409 Conflict");
  CHECK(webDrive("1 0", restartedGeneration).status == "200 OK");
  CHECK(webCommand("stop").status == "200 OK" && mode == Mode::Home);
  CHECK(webDrive("1 0", restartedGeneration).status == "409 Conflict");
  CHECK(webCommand("web").status == "200 OK");
  send("off");
  CHECK(webDrive("1 0", restartedGeneration).status == "409 Conflict");
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);

  Fake::now = UINT32_MAX - 100;
  CHECK(webCommand("web").status == "200 OK");
  const uint32_t wrappedGeneration = webGeneration;
  CHECK(webDrive("1 1", wrappedGeneration).status == "200 OK");
  Fake::now = webMotion.receivedMs + WEB_TIMEOUT_MS;
  loop();
  CHECK(mode == Mode::Home && webGeneration != wrappedGeneration);
  CHECK(webCommand("web").status == "200 OK");
  finishWebPose();
  CHECK(webDrive("1 0", webGeneration).status == "200 OK");
  Fake::writeFailure = true;
  Fake::now += settings.updateMs;
  loop();
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !servosReady);
  CHECK(webCommand("web").status == "400 Bad Request");
  Fake::writeFailure = false;
  CHECK(webCommand("init").status == "200 OK");
  loop();
  CHECK(servosReady && apActive && mode == Mode::Off);
}

void testWebRadioPriority()
{
  prepareWebTest();
  CHECK(webCommand("web").status == "200 OK");
  const uint32_t generation = webGeneration;
  CHECK(webDrive("1 -1", generation).status == "200 OK");
  receive("invalid");
  CHECK(apActive && mode == Mode::Web);
  ControlPacket stale;
  stale.receivedAtMs = millis() - settings.timeoutMs;
  xQueueOverwrite(controlQueue, &stale);
  loop();
  CHECK(apActive && mode == Mode::Web);
  receive("0,2047,0,2047,4095,2047,1,1,1,1");
  CHECK(!apActive && !Fake::apStarted && Fake::wifiMode == WIFI_STA);
  CHECK(mode == Mode::Radio && !control.armed && control.direction == 0 && control.rotation == 0);
  CHECK(webMotion.direction == 0 && webMotion.rotation == 0);
  CHECK(webDrive("1 -1", generation).status == "409 Conflict");
  CHECK(webCommand("off").status == "409 Conflict");
  receive("0,2047,2047,2047,2047,2047,1,1,1,1");
  CHECK(control.armed);
  receive("0,2047,0,2047,4095,2047,1,1,1,1");
  CHECK(control.direction == 1 && control.rotation == -1);
  Fake::now = control.lastPacketMs + settings.timeoutMs - 1;
  loop();
  CHECK(!apActive);
  loop();
  CHECK(apActive && Fake::apStarted && !control.armed && control.direction == 0);
  CHECK(webDrive("1 0", generation).status == "409 Conflict");
  CHECK(mode == Mode::Radio);
  CHECK(webCommand("web").status == "200 OK" && mode == Mode::Web);

  for (const char* protectedCommand : {"off", "servo rf coxa 90", "testservos", "home", "walk 1 0 10000"})
  {
    send("off");
    send(protectedCommand);
    const Mode protectedMode = mode;
    receive("0,2047,2047,2047,2047,2047,1,1,1,1");
    CHECK(mode == protectedMode && !apActive && !control.armed);
    receive("0,2047,0,2047,4095,2047,1,1,1,1");
    CHECK(mode == protectedMode && !control.armed);
  }
  send("off");
}

void testWebNetworkFailures()
{
  prepareWebTest();
  receive("0,2047,2047,2047,2047,2047,1,1,1,1");
  CHECK(!apActive);
  Fake::apFailure = true;
  Fake::now += settings.timeoutMs;
  loop();
  CHECK(!apActive && !Fake::apStarted && Serial.output.find("cannot start fallback AP") != std::string::npos);
  Fake::apFailure = false;
  loop();
  CHECK(!apActive);
  Fake::now = lastApAttemptMs + AP_RETRY_MS;
  loop();
  CHECK(apActive && Fake::apStarted);
  Fake::apStopFailure = true;
  receive("0,2047,2047,2047,2047,2047,1,1,1,1");
  CHECK(apActive && Serial.output.find("cannot stop fallback AP") != std::string::npos);
  CHECK(webCommand("web").status == "409 Conflict");
  Fake::apStopFailure = false;
  Fake::now += settings.timeoutMs;
  loop();
  CHECK(apActive && mode == Mode::Off);

  Fake::httpReceiveDelayMs = WEB_REQUEST_MAX_AGE_MS;
  CHECK(webCommand("web").status == "409 Conflict");
  CHECK(mode == Mode::Off);
  Fake::httpReceiveDelayMs = 0;
  const uint32_t generation = webGeneration;
  CHECK(http("/api/command", std::to_string(generation - 1) + "\nweb").status == "409 Conflict");
  CHECK(mode == Mode::Off);
  CHECK(webCommand("web").status == "200 OK");
  CHECK(http("/api/command", std::to_string(generation) + "\noff").status == "200 OK");
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);

  const auto timeout = http("/api/command", std::to_string(webGeneration) + "\nweb", true, false);
  CHECK(timeout.status == "504 Gateway Timeout");
  loop();
  CHECK(mode == Mode::Off);
  CHECK(webCommand("web").status == "200 OK");
  CHECK(webCommand("off").status == "200 OK");
  WebRequest queued;
  queued.id = UINT32_MAX;
  queued.receivedMs = millis();
  CHECK(xQueueSend(webTransport.requests, &queued, 0) == pdPASS);
  CHECK(webCommand("web").status == "503 Service Unavailable");
  loop();
  CHECK(mode == Mode::Off);

  CHECK(webCommand("set radio_channel 6").status == "200 OK");
  loop();
  CHECK(!wifiReady && !radioReady && !apActive);
  send("init");
  loop();
  CHECK(apActive && wifiReady && radioReady && Fake::channel == 6);
  send("defaults");
  send("init");
}

std::vector<Fake::PwmWrite> motionWrites()
{
  std::vector<Fake::PwmWrite> writes;
  for (const auto& write : Fake::pwmWrites)
    if (write.off != 4096)
      writes.push_back(write);
  return writes;
}

size_t activeServos()
{
  size_t count = 0;
  for (const auto& leg : lastPulses)
    for (uint16_t pulse : leg)
      if (pulse != 0)
        ++count;
  return count;
}

void checkWriteSpacing(uint32_t minimumMs)
{
  const auto writes = motionWrites();
  for (size_t i = 1; i < writes.size(); ++i)
    CHECK(uint32_t(writes[i].atMs - writes[i - 1].atMs) >= minimumMs);
}

void checkPhasedPulses()
{
  for (size_t leg = 0; leg < LEG_COUNT; ++leg)
    for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
    {
      const size_t board = servoBoard(leg);
      const uint8_t channel = servoChannel(leg, joint);
      if (lastPulses[leg][joint] == 0)
        CHECK(Fake::pwmOff[board][channel] == 4096);
      else
      {
        const uint16_t expectedOn = uint16_t((leg * JOINT_COUNT + joint) * 4096 / 18);
        const uint16_t ticks = pulseToTicks(lastPulses[leg][joint],
                                            settings.oscillatorHz(board), pwmPrescale[board]);
        CHECK(Fake::pwmOn[board][channel] == expectedOn);
        CHECK(Fake::pwmOff[board][channel] == (expectedOn + ticks) % 4096);
        CHECK(Fake::pwm[board][channel] == ticks);
      }
    }
  for (uint8_t channel = 0; channel < 7; ++channel)
    CHECK(Fake::pwmOff[0][channel] == 4096);
  for (uint8_t channel = 9; channel < 16; ++channel)
    CHECK(Fake::pwmOff[1][channel] == 4096);
}

void testStaggeredPoses()
{
  prepareWebTest();
  const struct { const char* command; size_t count; Mode expected; } cases[] =
  {
    {"center all", 18, Mode::Pose}, {"center rm", 3, Mode::Pose},
    {"servo rf coxa 95", 1, Mode::Pose}, {"pulse lf tibia 1600", 1, Mode::Pose},
    {"joints rr 85 90 95", 3, Mode::Pose}, {"foot lr 0 0 0", 3, Mode::Pose},
    {"home", 18, Mode::Home}, {"radio", 18, Mode::Radio}, {"walk 1 0 100", 18, Mode::Walk}
  };
  for (const auto& item : cases)
  {
    send("off");
    Fake::now += 250;
    Fake::pwmWrites.clear();
    const uint32_t started = millis();
    send(item.command, "\n", false);
    CHECK(posePending && mode == item.expected && activeServos() == 1);
    const float phase = gaitPhase;
    for (unsigned elapsed = 0; posePending && elapsed < 10000; ++elapsed)
    {
      const size_t before = Fake::pwmWrites.size();
      loop();
      CHECK(Fake::pwmWrites.size() - before <= 1);
      CHECK(gaitPhase == phase && mode == item.expected);
    }
    CHECK(!posePending && activeServos() == item.count);
    CHECK(motionWrites().size() == item.count);
    checkWriteSpacing(250);
    CHECK(uint32_t(millis() - started) >= item.count * 250);
    CHECK(uint32_t(millis() - started) <= item.count * 250 + 2);
    CHECK(memcmp(targetPulses, lastPulses, sizeof(lastPulses)) == 0);
    checkPhasedPulses();
    if (mode == Mode::Walk)
    {
      CHECK(uint32_t(millis() - manualMotion.startedMs) <= 1);
      Fake::now = manualMotion.startedMs + 99;
      loop();
      CHECK(mode == Mode::Walk);
      loop();
      CHECK(mode == Mode::Home);
    }
  }
  send("off");
}

void testPendingPoseCancellationAndTiming()
{
  prepareWebTest();
  Fake::now += 250;
  Fake::pwmWrites.clear();
  send("center all", "\n", false);
  CHECK(activeServos() == 1);
  const uint32_t first = motionWrites().front().atMs;
  send("joints rf 80 85 95", "\n", false);
  CHECK(motionWrites().size() == 1 && posePending);
  Fake::now = first + 249;
  loop();
  CHECK(motionWrites().size() == 1);
  loop();
  CHECK(motionWrites().size() == 2);
  CHECK(lastPulses[0][1] == 1472);
  send("stop", "\n", false);
  CHECK(mode == Mode::Pose && !posePending && activeServos() == 2);
  CHECK(Fake::outputEnable == LOW);
  const size_t stopped = Fake::pwmWrites.size();
  Fake::now += 10000;
  loop();
  CHECK(Fake::pwmWrites.size() == stopped && activeServos() == 2);

  send("home", "\n", false);
  CHECK(posePending && activeServos() <= 3);
  send("!", "\n", false);
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH && !posePending);
  const size_t disabled = Fake::pwmWrites.size();
  Fake::now += 10000;
  loop();
  CHECK(Fake::pwmWrites.size() == disabled && activeServos() == 0);
  send("servo lf tibia 90");
  CHECK(activeServos() == 1);
  checkPhasedPulses();
  send("off");

  Fake::now = UINT32_MAX - 100;
  Fake::pwmWrites.clear();
  send("center all", "\n", false);
  const uint32_t wrappedStart = motionWrites().front().atMs;
  Fake::now = wrappedStart + 249;
  loop();
  CHECK(motionWrites().size() == 1);
  loop();
  CHECK(motionWrites().size() == 2 && millis() < 1000);
  Fake::now += 10000;
  loop();
  CHECK(motionWrites().size() == 3);
  loop();
  CHECK(motionWrites().size() == 3);
  finishPose();
  CHECK(motionWrites().size() == 18);
  checkWriteSpacing(250);
  send("off");

  Fake::now += 250;
  send("walk 1 0 100", "\n", false);
  CHECK(mode == Mode::Walk && posePending);
  send("stop", "\n", false);
  CHECK(mode == Mode::Pose && !posePending && activeServos() == 1);
  Fake::now += 10000;
  loop();
  CHECK(mode == Mode::Pose && activeServos() == 1);
  send("off");

  Fake::now += 250;
  send("center all", "\n", false);
  Fake::writesUntilFailure = 1;
  Fake::now += 250;
  loop();
  CHECK(posePending && activeServos() == 2);
  Fake::now += 250;
  loop();
  CHECK(!posePending && mode == Mode::Off && !servosReady && Fake::outputEnable == HIGH);
  CHECK(Serial.output.find("I2C servo write failed") != std::string::npos);
  CHECK(activeServos() == 0);
  for (const auto& leg : targetPulses)
    for (uint16_t pulse : leg)
      CHECK(pulse == 0);
  Fake::writesUntilFailure = -1;
  send("init");
  Fake::now += 10000;
  loop();
  CHECK(mode == Mode::Off && Fake::outputEnable == HIGH);
}

void testStaggeredGaitAndPwm()
{
  prepareWebTest();
  for (unsigned hz : {40, 50, 60})
  {
    send("set servo_hz " + std::to_string(hz));
    send("init");
    send("home");
    Fake::pwmWrites.clear();
    send("walk 1 -0.5 10000");
    for (unsigned elapsed = 0; elapsed < 1000; ++elapsed)
    {
      const size_t before = Fake::pwmWrites.size();
      loop();
      CHECK(Fake::pwmWrites.size() - before <= 1);
    }
    CHECK(motionWrites().size() > 100);
    checkWriteSpacing(1);
    checkPhasedPulses();
    send("stop");
    for (unsigned elapsed = 0; elapsed < 1000; ++elapsed)
      loop();
    CHECK(mode == Mode::Home);
    checkWriteSpacing(1);
    checkPhasedPulses();
    const size_t held = Fake::pwmWrites.size();
    for (unsigned elapsed = 0; elapsed < 100; ++elapsed)
      loop();
    CHECK(Fake::pwmWrites.size() == held);
    send("off");
    send("cal lf tibia 500 2500 0 0");
    send("pulse lf tibia 2500");
    checkPhasedPulses();
    CHECK(Fake::pwmOff[1][8] < Fake::pwmOn[1][8]);
    send("off");
  }
  send("defaults");
  send("init");
}

void testWebStaggeringAndInitialization()
{
  prepareWebTest();
  Fake::now += 250;
  const uint32_t started = millis();
  CHECK(webCommand("center all").status == "200 OK");
  CHECK(uint32_t(millis() - started) < 10);
  CHECK(posePending && activeServos() == 1);
  CHECK(http("/api/status").response.find("\"posePending\":true") != std::string::npos);
  CHECK(webCommand("stop").status == "200 OK");
  CHECK(mode == Mode::Pose && !posePending && activeServos() == 1);
  CHECK(webCommand("off").status == "200 OK");
  Fake::pwmWrites.clear();
  Fake::driverOutputEnable.clear();
  Fake::radioOutputEnable.clear();
  Fake::radioActiveChannels.clear();
  CHECK(webCommand("init").status == "200 OK");
  CHECK(mode == Mode::Off && servosReady && radioReady && Fake::outputEnable == HIGH);
  CHECK(motionWrites().empty());
  CHECK(Fake::driverOutputEnable.size() == 2 && !Fake::radioOutputEnable.empty());
  for (int oe : Fake::driverOutputEnable)
    CHECK(oe == HIGH);
  for (int oe : Fake::radioOutputEnable)
    CHECK(oe == HIGH);
  for (unsigned active : Fake::radioActiveChannels)
    CHECK(active == 0);
  for (const auto& write : Fake::pwmWrites)
    CHECK(write.outputEnable == HIGH && write.off == 4096);
  loop();
  Fake::now += 250;
  CHECK(webCommand("web").status == "200 OK");
  CHECK(posePending && activeServos() == 1);
  CHECK(webDrive("1 1", webGeneration).status == "200 OK");
  const float phase = gaitPhase;
  Fake::now = webMotion.receivedMs + 500;
  loop();
  CHECK(mode == Mode::Home && posePending && gaitPhase == phase);
  CHECK(webMotion.direction == 0 && webMotion.rotation == 0);
  finishPose();
  CHECK(mode == Mode::Home && activeServos() == 18 && gaitPhase == phase);
  CHECK(http("/api/status").response.find("\"posePending\":false") != std::string::npos);
  CHECK(webCommand("off").status == "200 OK");
  CHECK(webCommand("web").status == "200 OK");
  CHECK(posePending);
  CHECK(webCommand("off").status == "200 OK");
  Fake::now += 10000;
  loop();
  CHECK(mode == Mode::Off && !posePending && activeServos() == 0 && Fake::outputEnable == HIGH);
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
  testWebCommands();
  testWebDrivingAndTimeout();
  testWebRadioPriority();
  testWebNetworkFailures();
  testStaggeredPoses();
  testPendingPoseCancellationAndTiming();
  testStaggeredGaitAndPwm();
  testWebStaggeringAndInitialization();
  puts("PASS: firmware console, staggered poses/gait/PWM, servo tests, NVS, web API, AP fallback, ESP-NOW priority and watchdog/fault recovery.");
}
