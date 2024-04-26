// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include <vector>

#include "mjlib/base/pid.h"
#include "mjlib/base/visitor.h"

#include "base/point3d.h"
#include "base/sophus.h"

namespace mjmech {
namespace mech {

// This represents the JSON used to configure the geometry of the
// robot.
struct HoverbotConfig {
  double period_s = 0.0025;
  double min_voltage = 16.0;

  double wheel_diameter_m = 0.163;

  struct Joint {
    int id = 0;
    double sign = 1.0;
    double rezero_pos_deg = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(sign));
      a->Visit(MJ_NVP(rezero_pos_deg));
    }
  };

  std::vector<Joint> joints;

  struct Pitch {
    double pitch_offset_deg = 0.0;
    mjlib::base::PID::Config pitch_pid;
    mjlib::base::PID::Config yaw_pid;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pitch_offset_deg));
      a->Visit(MJ_NVP(pitch_pid));
      a->Visit(MJ_NVP(yaw_pid));
    }
  };

  Pitch pitch;

  struct Drive {
    double pitch_limit_deg = 20.0;
    mjlib::base::PID::Config drive_pid;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pitch_limit_deg));
      a->Visit(MJ_NVP(drive_pid));
    }
  };

  Drive drive;

  double voltage_filter_s = 1.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(period_s));
    a->Visit(MJ_NVP(min_voltage));
    a->Visit(MJ_NVP(wheel_diameter_m));
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(pitch));
    a->Visit(MJ_NVP(drive));
    a->Visit(MJ_NVP(voltage_filter_s));
  }
};

}
}
