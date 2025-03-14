#pragma once

#include <cstdint>

enum class CommandCode : uint8_t {
  NONE = 0,  // expects 0 data
  STOP,      // expects 0 data
  SET_ANGLE, // expects 1 float in radians
  SET_SPEED, // expects 1 float in radians per second
};

struct Command {
  CommandCode code{CommandCode::NONE};
  union {
    float angle_radians;
    float speed_radians_per_second;
    uint8_t data[4] = {0};
  } __attribute__((packed));

  Command() = default;

  static constexpr float MIN_ANGLE_DIFF = 0.05f;
  static constexpr float MIN_SPEED_DIFF = 0.1f;

  bool operator==(const Command &other) const {
    if (code != other.code) {
      return false;
    }
    switch (code) {
    case CommandCode::NONE:
    case CommandCode::STOP:
      return true;
    case CommandCode::SET_ANGLE:
      return fabs(angle_radians - other.angle_radians) < MIN_ANGLE_DIFF;
    case CommandCode::SET_SPEED:
      return fabs(speed_radians_per_second - other.speed_radians_per_second) < MIN_SPEED_DIFF;
    }
    return false;
  }

  size_t size() const {
    switch (code) {
    case CommandCode::NONE:
    case CommandCode::STOP:
      return 1;
    case CommandCode::SET_ANGLE:
      return 5;
    case CommandCode::SET_SPEED:
      return 5;
    }
    return 0;
  }

} __attribute__((packed));
