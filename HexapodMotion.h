#pragma once

#include "HexapodControl.h"

/*
Walking phase tables and swing/stance design adapted from Christoph Streit's Hexapod.
MIT License
Copyright (c) 2026 Christoph Streit

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

namespace Hexapod
{
constexpr double MOTION_PI = 3.14159265358979323846;

struct FootTarget
{
  float x, y, z;
  FootTarget(float px = 0, float py = 0, float pz = 0) : x(px), y(py), z(pz) {}
};

inline float motionClamp(float value, float low, float high)
{
  return fminf(high, fmaxf(low, value));
}

inline float smoothFraction(float t)
{
  t = motionClamp(t, 0, 1);
  return t * t * t * (t * (t * 6 - 15) + 10);
}

inline float motionFilter(float current, float target, float dtMs)
{
  return target + (current - target) * expf(-dtMs / 250.0f);
}

struct WalkingPattern
{
  float swing;
  float offsets[LEG_COUNT];
  float speedLimit;
};

// RF, RM, RR, LR, LM, LF (not the donor's left-side-first order).
constexpr WalkingPattern TRIPOD_PATTERN = {0.5f, {0.5f, 0, 0.5f, 0, 0.5f, 0}, 1.0f};
constexpr WalkingPattern TETRAPOD_PATTERN =
    {1.0f / 3, {1.0f / 3, 2.0f / 3, 0, 2.0f / 3, 1.0f / 3, 0}, 0.7f};
constexpr WalkingPattern RIPPLE_PATTERN =
    {0.32f, {3.0f / 6, 5.0f / 6, 1.0f / 6, 4.0f / 6, 2.0f / 6, 0}, 0.7f};

inline const WalkingPattern* walkingPattern(Gait gait)
{
  switch (gait)
  {
    case Gait::Tripod: return &TRIPOD_PATTERN;
    case Gait::Tetrapod: return &TETRAPOD_PATTERN;
    case Gait::Ripple: return &RIPPLE_PATTERN;
    default: return nullptr;
  }
}

inline FootTarget stridePoint(const FootTarget& home, const MotionInput& stride,
                              float yaw, float fraction)
{
  const float c = cosf(yaw * fraction), s = sinf(yaw * fraction);
  return {home.x * c - home.y * s + stride.forward * fraction,
          home.x * s + home.y * c + stride.strafe * fraction, home.z};
}

inline FootTarget walkingFoot(float phase, size_t leg, const WalkingPattern& pattern,
                              const FootTarget& home, const MotionInput& stride,
                              float yaw, float lift)
{
  const float p = fmodf(phase + pattern.offsets[leg], 1.0f);
  if (p < pattern.swing)
  {
    const float u = p / pattern.swing;
    const float t = 0.5f * (1.0f - cosf(float(MOTION_PI) * u));
    const float along = t * t * (3 - 2 * t);
    const FootTarget start = stridePoint(home, stride, yaw, -0.5f);
    const FootTarget end = stridePoint(home, stride, yaw, 0.5f);
    // Finish touchdown during swing: PWM servos provide no contact feedback.
    const float z = u < 0.5f ? lift * smoothFraction(2 * u) :
        lift * (1 - smoothFraction(2 * u - 1));
    return {start.x + (end.x - start.x) * along,
            start.y + (end.y - start.y) * along, home.z + z};
  }
  const float t = (p - pattern.swing) / (1 - pattern.swing);
  return stridePoint(home, stride, yaw, 0.5f - t);
}

class MotionEngine
{
  MotionInput filtered, stopStart;
  float demand = 0, stopDemand = 0, stopMs = 0;
  float liftDemand = 0, stopLift = 0;
  float phaseValue = 0, rate = 0, stride = 0, lift = 0, turn = 0, groundZ = 0;
  bool resting = true, stopping = false;

public:
  static constexpr float MAX_DT_MS = 50;
  static constexpr float STOP_MS = 1200;

  void reset(const Settings& settings)
  {
    filtered = stopStart = MotionInput{};
    demand = stopDemand = stopMs = phaseValue = 0;
    liftDemand = stopLift = 0;
    rate = 1.0f / (float(settings.motionCycleMs) * 4);
    stride = settings.motionStride;
    lift = settings.motionLift;
    turn = settings.motionTurn;
    groundZ = settings.homeZ;
    resting = true;
    stopping = false;
  }

  bool idle() const { return resting; }
  bool settling() const { return stopping; }
  float phase() const { return phaseValue; }
  bool atHome(const Settings& settings) const { return resting && groundZ == settings.homeZ; }

  bool update(float elapsedMs, MotionInput input, Gait gait, const Settings& settings,
              const FootTarget (&homes)[LEG_COUNT], FootTarget (&targets)[LEG_COUNT])
  {
    const WalkingPattern* pattern = walkingPattern(gait);
    if (pattern == nullptr || !isfinite(elapsedMs) || elapsedMs < 0 ||
        !isfinite(input.forward) || !isfinite(input.strafe) || !isfinite(input.turn) ||
        fabsf(input.forward) > 1 || fabsf(input.strafe) > 1 || fabsf(input.turn) > 1)
      return false;

    const float dt = fminf(elapsedMs, MAX_DT_MS);
    const float length = hypotf(input.forward, input.strafe);
    if (length > 1)
    {
      input.forward /= length;
      input.strafe /= length;
    }
    const float requested = fmaxf(fminf(length, 1), fabsf(input.turn));
    if (requested > 0)
    {
      resting = stopping = false;
      filtered.forward = motionFilter(filtered.forward, input.forward, dt);
      filtered.strafe = motionFilter(filtered.strafe, input.strafe, dt);
      filtered.turn = motionFilter(filtered.turn, input.turn, dt);
      // Independent demand bridges a direction reversal without dropping all the feet.
      demand = motionFilter(demand, requested, dt);
      liftDemand = motionFilter(liftDemand, smoothFraction(fminf(requested / 0.2f, 1)), dt);
    }
    else if (!resting)
    {
      if (!stopping)
      {
        stopping = true;
        stopStart = filtered;
        stopDemand = demand;
        stopLift = liftDemand;
        stopMs = 0;
      }
      stopMs = fminf(stopMs + dt, STOP_MS);
      const float remaining = 1 - smoothFraction(stopMs / STOP_MS);
      filtered = {stopStart.forward * remaining, stopStart.strafe * remaining,
                  stopStart.turn * remaining};
      demand = stopDemand * remaining;
      liftDemand = stopLift * remaining;
      if (stopMs == STOP_MS)
      {
        resting = true;
        stopping = false;
        phaseValue = 0;
      }
    }

    stride = motionFilter(stride, settings.motionStride, dt);
    lift = motionFilter(lift, settings.motionLift, dt);
    turn = motionFilter(turn, settings.motionTurn, dt);
    groundZ += motionClamp(settings.homeZ - groundZ, -20 * dt / 1000, 20 * dt / 1000);
    const float fastest = float(settings.motionCycleMs) / pattern->speedLimit;
    if (!stopping)
    {
      const float shaped = demand * (0.4f + 0.6f * demand * demand);
      rate = motionFilter(rate, (0.25f + 0.75f * shaped) / fastest, dt);
    }
    rate = motionClamp(rate, 0.25f / fastest, 1.0f / fastest);
    if (!resting)
      phaseValue = fmodf(phaseValue + dt * rate, 1.0f);

    const MotionInput travel{filtered.forward * stride, filtered.strafe * stride, 0};
    const float yaw = filtered.turn * turn * float(MOTION_PI / 180);
    const float footLift = lift * liftDemand;
    for (size_t leg = 0; leg < LEG_COUNT; ++leg)
    {
      FootTarget home = homes[leg];
      home.z = groundZ;
      targets[leg] = resting ? home : walkingFoot(phaseValue, leg, *pattern, home, travel, yaw, footLift);
    }
    return true;
  }
};

inline bool reachableFoot(const FootTarget& target, const FootTarget& origin, const Settings& settings)
{
  if (!isfinite(target.x) || !isfinite(target.y) || !isfinite(target.z))
    return false;
  const float forward = hypotf(target.x - origin.x, target.y - origin.y) - settings.coxaLength;
  const float distance = hypotf(forward, target.z);
  return distance > fabsf(settings.femurLength - settings.tibiaLength) + 0.001f &&
         distance < settings.femurLength + settings.tibiaLength - 0.001f;
}

inline bool withinJointLimits(const FootTarget& angles, size_t leg, const Settings& settings)
{
  const float values[] = {90 + angles.x, 90 + angles.y, fabsf(angles.z)};
  if (values[0] < settings.coxaMin || values[0] > settings.coxaMax)
    return false;
  for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
  {
    const ServoCalibration& calibration = settings.servos[leg][joint];
    const float adjusted = (calibration.reversed ? 180 - values[joint] : values[joint]) +
                           calibration.trimDegrees;
    if (!isfinite(values[joint]) || values[joint] < 0 || values[joint] > 180 ||
        adjusted < 0 || adjusted > 180)
      return false;
  }
  return true;
}
}
