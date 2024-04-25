// Copyright 2020 Josh Pieper, jjp@pobox.com.
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
#include <string>

#include <boost/asio/any_io_executor.hpp>
#include <boost/signals2.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"
#include "mjlib/multiplex/asio_client.h"
#include "mjlib/multiplex/register.h"

#include "mech/attitude_data.h"
#include "mech/pi3hat_interface.h"

namespace mjmech {
namespace mech {

/// Provide an interface to the pi3hat.
///
/// NOTE: This treats the AsioClient as the primary interface.  IMU
/// and RF requests will only be serviced when AsyncTransmit is
/// called.
class Pi3hatWrapper : public Pi3hatInterface {
 public:
  struct Mounting {
    double yaw_deg = 180.0;
    double pitch_deg = 90.0;
    double roll_deg = -90.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(yaw_deg));
      a->Visit(MJ_NVP(pitch_deg));
      a->Visit(MJ_NVP(roll_deg));
    }
  };

  struct Options {
    // If set to a non-negative number, bind the time sensitive thread
    // to the given CPU.
    int cpu_affinity = -1;

    int spi_speed_hz = 10000000;

    // When waiting for CAN data, wait this long before timing out.
    double query_timeout_s = 0.001;

    // And guarantee to wait at least this long after any successful
    // receives to catch stragglers. (Mostly to prevent stale data
    // from the previous cycle causing us to be one cycle behind).
    double min_wait_s = 0.00005;

    Mounting mounting;
    uint32_t rf_id = 5678;
    double power_poll_period_s = 0.1;
    double shutdown_timeout_s = 15.0;
    uint32_t imu_rate_hz = 400;
    bool attitude_detail = false;
    int force_bus = -1;

    int power_dist_rev = 0x0403;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(cpu_affinity));
      a->Visit(MJ_NVP(spi_speed_hz));
      a->Visit(MJ_NVP(query_timeout_s));
      a->Visit(MJ_NVP(mounting));
      a->Visit(MJ_NVP(rf_id));
      a->Visit(MJ_NVP(power_poll_period_s));
      a->Visit(MJ_NVP(shutdown_timeout_s));
      a->Visit(MJ_NVP(imu_rate_hz));
      a->Visit(MJ_NVP(attitude_detail));
      a->Visit(MJ_NVP(force_bus));
      a->Visit(MJ_NVP(power_dist_rev));
    }
  };

  Pi3hatWrapper(const boost::asio::any_io_executor&, const Options&);
  ~Pi3hatWrapper();

  void AsyncStart(mjlib::io::ErrorCallback);

  // ************************
  // mp::AsioClient

  /// Request a request be made to one or more servos (and optionally
  /// have a reply sent back).
  void AsyncTransmit(const Request*,
                     Reply*,
                     mjlib::io::ErrorCallback) override;

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) override;

  struct Power {
    boost::posix_time::ptime timestamp;

    double output_V = 0.0;
    double output_A = 0.0;
    double power_W = 0.0;
    double avg_power_W = 0.0;
    double temp_C = 0.0;
    double energy_Whr = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(output_V));
      a->Visit(MJ_NVP(output_A));
      a->Visit(MJ_NVP(power_W));
      a->Visit(MJ_NVP(avg_power_W));
      a->Visit(MJ_NVP(temp_C));
      a->Visit(MJ_NVP(energy_Whr));
    }
  };
  using PowerSignal = boost::signals2::signal<void (const Power*)>;
  PowerSignal* power_signal();

  struct Stats {
    template <typename Archive>
    void Serialize(Archive* a) {
    }
  };
  Stats stats() const;

  // ************************
  // ImuClient

  /// Read the next available IMU sample and store it in @p data.
  ///
  /// @param callback will be invoked when the operation completes or
  /// fails.
  void ReadImu(AttitudeData* data, mjlib::io::ErrorCallback callback) override;

  void Cycle(AttitudeData*,
             const Request* request,
             Reply* reply,
             mjlib::io::ErrorCallback callback) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
