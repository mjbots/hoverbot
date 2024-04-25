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

#include <memory>

#include <clipp/clipp.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/noncopyable.hpp>

#include "mjlib/base/visitor.h"

#include "base/context.h"

#include "mech/control_timing.h"
#include "mech/pi3hat_interface.h"
#include "mech/hoverbot_command.h"
#include "mech/hoverbot_state.h"

namespace mjmech {
namespace mech {

/// This sequences the primary control modes of the quadruped.
class HoverbotControl : boost::noncopyable {
 public:
  /// @param client_getter will be called at AsyncStart time
  using Pi3hatGetter = std::function<Pi3hatInterface*()>;
  HoverbotControl(base::Context&, Pi3hatGetter client_getter);
  ~HoverbotControl();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  struct Parameters {
    double max_torque_Nm = -1.0;
    std::string config;
    std::string log_filename_base = "mjbots-hoverbot.log";

    bool enable_imu = true;
    bool servo_debug = false;

    double command_timeout_s = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(config));
      a->Visit(MJ_NVP(log_filename_base));
      a->Visit(MJ_NVP(enable_imu));
      a->Visit(MJ_NVP(servo_debug));
      a->Visit(MJ_NVP(command_timeout_s));
    }
  };

  struct Status {
    boost::posix_time::ptime timestamp;

    HoverbotCommand::Mode mode = HoverbotCommand::Mode::kConfiguring;
    boost::posix_time::ptime mode_start;
    std::string fault;

    HoverbotState state;

    int missing_replies = 0;
    ControlTiming::Status timing;
    bool performed_rezero = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(mode_start));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(state));
      a->Visit(MJ_NVP(missing_replies));
      a->Visit(MJ_NVP(timing));
      a->Visit(MJ_NVP(performed_rezero));
    }
  };

  struct ControlLog {
    using HC = HoverbotCommand;

    boost::posix_time::ptime timestamp;

    std::vector<HC::Joint> joints;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(joints));
    }
  };

  void Command(const HoverbotCommand&);
  const Status& status() const;

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
