#include "../HexapodMotion.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <initializer_list>

using namespace Hexapod;

void check(bool value, const char* expression, int line)
{
  if (!value)
  {
    fprintf(stderr, "FAIL line %d: %s\n", line, expression);
    exit(1);
  }
}
#define CHECK(value) check((value), #value, __LINE__)

float distance(const FootTarget& a, const FootTarget& b)
{
  return hypotf(hypotf(a.x - b.x, a.y - b.y), a.z - b.z);
}

void homesFor(const Settings& settings, FootTarget (&homes)[LEG_COUNT])
{
  const float reach = settings.coxaLength +
      (settings.femurLength + settings.tibiaLength) * float(cos(MOTION_PI / 4));
  for (size_t i = 0; i < LEG_COUNT; ++i)
  {
    const float yaw = atan2f(settings.origins[i].y, settings.origins[i].x);
    homes[i] = {settings.origins[i].x + reach * cosf(yaw),
                settings.origins[i].y + reach * sinf(yaw), settings.homeZ};
  }
}

void testPatternsAndTrajectories()
{
  CHECK(walkingPattern(Gait::Legacy) == nullptr);
  CHECK(walkingPattern(Gait(999)) == nullptr);
  const Settings settings;
  FootTarget homes[LEG_COUNT];
  homesFor(settings, homes);
  CHECK(TRIPOD_PATTERN.offsets[0] == 0.5f && TRIPOD_PATTERN.offsets[5] == 0);
  CHECK(TETRAPOD_PATTERN.offsets[2] == 0 && TETRAPOD_PATTERN.offsets[5] == 0);
  CHECK(RIPPLE_PATTERN.offsets[1] == 5.0f / 6 && RIPPLE_PATTERN.offsets[3] == 4.0f / 6);
  for (Gait gait : {Gait::Tripod, Gait::Tetrapod, Gait::Ripple})
  {
    const WalkingPattern& pattern = *walkingPattern(gait);
    size_t minimumSupport = LEG_COUNT;
    for (unsigned sample = 0; sample < 2000; ++sample)
    {
      const float phase = (float(sample) + 0.5f) / 2000;
      size_t support = 0;
      FootTarget planted[LEG_COUNT];
      for (size_t leg = 0; leg < LEG_COUNT; ++leg)
      {
        const FootTarget foot = walkingFoot(phase, leg, pattern, homes[leg], {40, 20, 0}, 0.1f, 25);
        CHECK(foot.z >= settings.homeZ - 0.0001f && foot.z <= settings.homeZ + 25.0001f);
        if (foot.z <= settings.homeZ + 0.0001f)
        {
          planted[support] = foot;
          ++support;
        }
        const FootTarget origin{settings.origins[leg].x, settings.origins[leg].y, 0};
        CHECK(reachableFoot(foot, origin, settings));
      }
      if (support < minimumSupport)
        minimumSupport = support;
      for (size_t i = 0; i < support; ++i)
      {
        const FootTarget& a = planted[i];
        const FootTarget& b = planted[(i + 1) % support];
        const float cross = (b.x - a.x) * -a.y - (b.y - a.y) * -a.x;
        CHECK(-cross / hypotf(b.x - a.x, b.y - a.y) > 10);
      }
    }
    CHECK(minimumSupport == (gait == Gait::Tripod ? 3u : 4u));
    for (size_t leg = 0; leg < LEG_COUNT; ++leg)
    {
      const float peak = fmodf(1 + pattern.swing / 2 - pattern.offsets[leg], 1);
      CHECK(fabsf(walkingFoot(peak, leg, pattern, homes[leg], {40, 0, 0}, 0, 25).z -
                  (homes[leg].z + 25)) < 0.0001f);
      for (float boundary : {0.0f, pattern.swing})
      {
        const float p = fmodf(1 + boundary - pattern.offsets[leg], 1);
        const FootTarget before = walkingFoot(fmodf(p + 1 - 0.00001f, 1), leg, pattern,
                                               homes[leg], {40, 20, 0}, 0.1f, 25);
        const FootTarget after = walkingFoot(fmodf(p + 0.00001f, 1), leg, pattern,
                                              homes[leg], {40, 20, 0}, 0.1f, 25);
        CHECK(distance(before, after) < 0.02f);
      }
      // Ground travel is linear, independent of where in stance each leg happens to be.
      FootTarget previous;
      for (unsigned n = 0; n < 5; ++n)
      {
        const float t = 0.1f + float(n) * 0.15f;
        const float phase = fmodf(1 + pattern.swing + t * (1 - pattern.swing) - pattern.offsets[leg], 1);
        const FootTarget foot = walkingFoot(phase, leg, pattern, homes[leg], {40, 20, 0}, 0, 25);
        CHECK(foot.z == homes[leg].z);
        if (n != 0)
        {
          CHECK(fabsf(foot.x - previous.x + 6) < 0.001f);
          CHECK(fabsf(foot.y - previous.y + 3) < 0.001f);
        }
        previous = foot;
      }
    }
  }
}

void testEngine()
{
  Settings settings;
  FootTarget homes[LEG_COUNT], feet[LEG_COUNT], previous[LEG_COUNT];
  homesFor(settings, homes);
  for (Gait gait : {Gait::Tripod, Gait::Tetrapod, Gait::Ripple})
    for (MotionInput input : {MotionInput{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                             {0, 0, 1}, {0, 0, -1}, {1, 1, 1}, {-1, -1, -1}, {0.1f, 0, 0}})
    {
      MotionEngine engine;
      engine.reset(settings);
      CHECK(engine.update(0, {}, gait, settings, homes, feet) && engine.idle());
      for (size_t leg = 0; leg < LEG_COUNT; ++leg)
        CHECK(distance(feet[leg], homes[leg]) == 0);
      for (unsigned ms = 0; ms < 6000; ms += 10)
      {
        for (size_t leg = 0; leg < LEG_COUNT; ++leg)
          previous[leg] = feet[leg];
        CHECK(engine.update(10, input, gait, settings, homes, feet));
        for (size_t leg = 0; leg < LEG_COUNT; ++leg)
        {
          if (distance(feet[leg], previous[leg]) >= 5)
            fprintf(stderr, "gait=%u input=(%g,%g,%g) leg=%zu ms=%u delta=%g z=%g -> %g\n",
                unsigned(gait), input.forward, input.strafe, input.turn, leg, ms,
                distance(feet[leg], previous[leg]), previous[leg].z, feet[leg].z);
          CHECK(distance(feet[leg], previous[leg]) < 5);
          CHECK(reachableFoot(feet[leg], {settings.origins[leg].x, settings.origins[leg].y, 0}, settings));
        }
      }
      // Reverse all axes without collapsing the lift as filtered stride crosses zero.
      MotionEngine continuing = engine;
      const MotionInput original = input;
      input = {-input.forward, -input.strafe, -input.turn};
      for (unsigned ms = 0; ms < 400; ms += 10)
      {
        FootTarget uninterrupted[LEG_COUNT];
        CHECK(continuing.update(10, original, gait, settings, homes, uninterrupted));
        CHECK(engine.update(10, input, gait, settings, homes, feet));
        for (size_t leg = 0; leg < LEG_COUNT; ++leg)
          CHECK(fabsf(feet[leg].z - uninterrupted[leg].z) < 0.0001f);
      }
      for (unsigned ms = 0; ms < unsigned(MotionEngine::STOP_MS); ms += 10)
      {
        for (size_t leg = 0; leg < LEG_COUNT; ++leg)
          previous[leg] = feet[leg];
        CHECK(engine.update(10, {}, gait, settings, homes, feet));
        for (size_t leg = 0; leg < LEG_COUNT; ++leg)
          CHECK(distance(feet[leg], previous[leg]) < 5);
      }
      CHECK(engine.atHome(settings) && !engine.settling() && engine.phase() == 0);
      for (size_t leg = 0; leg < LEG_COUNT; ++leg)
        CHECK(distance(feet[leg], homes[leg]) == 0);
    }
  MotionEngine engine;
  engine.reset(settings);
  CHECK(!engine.update(10, {2, 0, 0}, Gait::Tripod, settings, homes, feet));
  CHECK(!engine.update(-1, {}, Gait::Tripod, settings, homes, feet));
  CHECK(!engine.update(std::numeric_limits<float>::quiet_NaN(), {}, Gait::Tripod, settings, homes, feet));
  CHECK(!engine.update(10, {}, Gait::Legacy, settings, homes, feet));
}

void testTimingAndTuning()
{
  Settings settings;
  FootTarget homes[LEG_COUNT], feet[LEG_COUNT], reference[LEG_COUNT];
  homesFor(settings, homes);
  float phase = 0;
  for (unsigned dt : {5u, 10u, 50u})
  {
    MotionEngine engine;
    engine.reset(settings);
    for (unsigned ms = 0; ms < 6000; ms += dt)
      CHECK(engine.update(float(dt), {1, 0, 0.5f}, Gait::Ripple, settings, homes, feet));
    if (dt == 5)
    {
      phase = engine.phase();
      for (size_t i = 0; i < LEG_COUNT; ++i)
        reference[i] = feet[i];
    }
    else
    {
      CHECK(fabsf(engine.phase() - phase) < 0.025f);
      for (size_t i = 0; i < LEG_COUNT; ++i)
        CHECK(distance(feet[i], reference[i]) < 4);
    }
    MotionEngine delayed = engine;
    CHECK(engine.update(50, {1, 0, 0.5f}, Gait::Ripple, settings, homes, feet));
    FootTarget delayedFeet[LEG_COUNT];
    CHECK(delayed.update(3600000, {1, 0, 0.5f}, Gait::Ripple, settings, homes, delayedFeet));
    for (size_t i = 0; i < LEG_COUNT; ++i)
      CHECK(distance(feet[i], delayedFeet[i]) == 0);
  }
  MotionEngine engine;
  engine.reset(settings);
  settings.homeZ = -60;
  CHECK(engine.update(50, {}, Gait::Tripod, settings, homes, feet));
  CHECK(feet[0].z == -41);
  CHECK(!engine.atHome(settings));
  for (unsigned ms = 50; ms < 1500; ms += 50)
    CHECK(engine.update(50, {}, Gait::Tripod, settings, homes, feet));
  CHECK(engine.atHome(settings) && feet[0].z == -60);
}

int main()
{
  testPatternsAndTrajectories();
  testEngine();
  testTimingAndTuning();
  puts("PASS: gait patterns, support, exact lift, continuous trajectories, proportional motion, reversal, stopping, tuning and timing.");
}
