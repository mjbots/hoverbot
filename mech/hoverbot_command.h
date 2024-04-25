// Copyright 2019 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <map>
#include <optional>
#include <vector>

#include "base/point3d.h"
#include "base/sophus.h"

namespace mjmech {
namespace mech {

struct HoverbotCommand {
  /// Higher values of priority take precedence.  A lower value
  /// command is only used in place of a higher value if the higher
  /// value is stale.
  int priority = 0;

  /// Determines if on-disk logging takes place.
  enum Log {
    // kUnset leaves the default command-line option in place.
    kUnset,

    kDisable,
    kEnable,
  };

  Log log = kUnset;

  enum Mode {
    // This is a transient state that should not be commanded.  The
    // quadruped uses it to perform initialization functions.
    kConfiguring = 0,

    // In this mode, all servos are powered off.
    kStopped = 1,

    // In this mode, all servos are set to zero velocity.  It is a
    // latched state.  The only valid transition from this state is to
    // kStopped.
    kFault = 2,

    // In this mode, all servos are set to zero velocity.  This is the
    // safest thing that can be done with no knowledge of the current
    // robot state.
    kZeroVelocity = 3,

    // Drive at a fixed velocity and yaw rate.
    kVelocity = 4,

    kNumModes,
  };

  Mode mode = kStopped;

  struct Joint {
    int id = 0;
    bool power = false;
    bool zero_velocity = false;
    double angle_deg = 0.0;
    double velocity_dps = 0.0;
    double torque_Nm = 0.0;
    std::optional<double> kp_scale;
    std::optional<double> kd_scale;
    std::optional<double> max_torque_Nm;
    std::optional<double> stop_angle_deg;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(power));
      a->Visit(MJ_NVP(zero_velocity));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(kp_scale));
      a->Visit(MJ_NVP(kd_scale));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(stop_angle_deg));
    }
  };

  // Only valid for kJoint mode.
  std::vector<Joint> joints;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(priority));
    a->Visit(MJ_NVP(log));
    a->Visit(MJ_NVP(mode));
    a->Visit(MJ_NVP(joints));
  }
};

}
}

namespace mjlib {
namespace base {
template <>
struct IsEnum<mjmech::mech::HoverbotCommand::Mode> {
  static constexpr bool value = true;

  using M = mjmech::mech::HoverbotCommand::Mode;

  static std::map<M, const char*> map() {
    return { {
        { M::kConfiguring, "configuring" },
        { M::kStopped, "stopped" },
        { M::kFault, "fault" },
        { M::kZeroVelocity, "zero_velocity" },
        { M::kVelocity, "velocity" },
      }};
  }
};

template <>
struct IsEnum<mjmech::mech::HoverbotCommand::Log> {
  static constexpr bool value = true;

  using M = mjmech::mech::HoverbotCommand::Log;

  static std::map<M, const char*> map() {
    return { {
        { M::kUnset, "unset" },
        { M::kDisable, "disable" },
        { M::kEnable, "enable" },
      }};
  }
};
}
}
