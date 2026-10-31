#pragma once
#include "Arduino.h"
inline uint32_t esp_random()
{
  static uint32_t value = 0x12345678;
  return ++value;
}
