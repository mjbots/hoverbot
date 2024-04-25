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

#include <deque>

#include <boost/noncopyable.hpp>

#include "mjlib/base/assert.h"

#include "mech/hoverbot_command.h"
#include "mech/hoverbot_config.h"
#include "mech/hoverbot_state.h"

namespace mjmech {
namespace mech {

struct HoverbotContext : boost::noncopyable {
  using Config = HoverbotConfig;
  using HC = HoverbotCommand;

  HoverbotContext(const HoverbotConfig& config_in,
                  const HoverbotCommand* command_in,
                   HoverbotState* state_in)
      : config(config_in),
        command(command_in),
        state(state_in) {
  }


  const HoverbotConfig& config;
  const HoverbotCommand* const command;
  HoverbotState* const state;
};

}
}
