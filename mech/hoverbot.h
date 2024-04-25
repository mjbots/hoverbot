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

#include <boost/noncopyable.hpp>

#include "mjlib/io/selector.h"

#include "base/component_archives.h"
#include "base/context.h"

#include "mech/pi3hat_interface.h"
#include "mech/hoverbot_control.h"
#include "mech/system_info.h"
#include "mech/web_control.h"

namespace mjmech {
namespace mech {

class Hoverbot : boost::noncopyable {
 public:
  Hoverbot(base::Context& context);
  ~Hoverbot();

  void AsyncStart(mjlib::io::ErrorCallback);

  using HoverbotWebControl =
      WebControl<HoverbotCommand, HoverbotControl::Status>;

  struct Members {
    std::unique_ptr<
      mjlib::io::Selector<Pi3hatInterface>> pi3hat;
    std::unique_ptr<HoverbotControl> hoverbot_control;
    std::unique_ptr<HoverbotWebControl> web_control;
    std::unique_ptr<SystemInfo> system_info;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pi3hat));
      a->Visit(MJ_NVP(hoverbot_control));
      a->Visit(MJ_NVP(web_control));
      a->Visit(MJ_NVP(system_info));
    }
  };

  Members* m();

  struct Parameters {
    template <typename Archive>
    void Serialize(Archive* a) {
    }
  };

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
