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

#include "mech/hoverbot_control.h"

#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/asio/post.hpp>

#include <fmt/format.h>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/json5_read_archive.h"

#include "mjlib/io/now.h"
#include "mjlib/io/repeating_timer.h"

#include "base/common.h"
#include "base/fit_plane.h"
#include "base/interpolate.h"
#include "base/logging.h"
#include "base/sophus.h"
#include "base/telemetry_registry.h"
#include "base/timestamped_log.h"

#include "mech/attitude_data.h"
#include "mech/moteus.h"
#include "mech/hoverbot_config.h"
#include "mech/hoverbot_context.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
constexpr int kNumServos = 2;

using HC = HoverbotCommand;
using HM = HC::Mode;

template <typename Iter, typename Functor>
auto Average(Iter begin, Iter end, Functor f) -> decltype(f(*begin)) {
  using T = decltype(f(*begin));

  T sum = {};
  double count = {};
  for (Iter it = begin; it != end; ++it) {
    count++;
    sum += f(*it);
  }
  return (1.0 / count) * sum;
}

template <typename Iter, typename Functor>
auto Max(Iter begin, Iter end, Functor f) -> decltype(f(*begin)) {
  using T = decltype(f(*begin));

  MJ_ASSERT(begin != end);

  T current = f(*begin);
  Iter it = begin;
  ++it;
  for (; it != end; ++it) {
    const auto value = f(*it);
    if (value > current) { current = value; }
  }
  return current;
}

template <typename Iter, typename Functor>
auto Min(Iter begin, Iter end, Functor f) -> decltype(f(*begin)) {
  using T = decltype(f(*begin));

  MJ_ASSERT(begin != end);

  T current = f(*begin);
  Iter it = begin;
  ++it;
  for (; it != end; ++it) {
    const auto value = f(*it);
    if (value < current) { current = value; }
  }
  return current;
}

struct ReportedServoConfig {
  boost::posix_time::ptime timestamp;

  struct Servo {
    int id = 0;
    int rezero_state = 0;
    int register_map_version = -1;
    std::array<uint8_t, 12> serial_number = {};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(rezero_state));
      a->Visit(MJ_NVP(register_map_version));
      a->Visit(MJ_NVP(serial_number));
    }
  };

  std::vector<Servo> servos;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(servos));
  }
};

using Config = HoverbotConfig;

struct CommandLog {
  boost::posix_time::ptime timestamp;

  const HC* command = &ignored_command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    const_cast<HC*>(command)->Serialize(a);
  }

  static HC ignored_command;
};

HC CommandLog::ignored_command;
}

class HoverbotControl::Impl {
 public:
  Impl(base::Context& context,
       Pi3hatGetter pi3hat_getter)
      : executor_(context.executor),
        telemetry_log_(context.telemetry_log.get()),
        timer_(executor_),
        pi3hat_getter_(pi3hat_getter) {
    context.telemetry_registry->Register("hc_status", &status_signal_);
    context.telemetry_registry->Register("hc_command", &command_signal_);
    context.telemetry_registry->Register("hc_control", &control_signal_);
    context.telemetry_registry->Register("imu", &imu_signal_);
    context.telemetry_registry->Register("servo_config", &servo_config_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    pi3hat_ = pi3hat_getter_();

    BOOST_ASSERT(!!pi3hat_);

    // Load our configuration.
    std::vector<std::string> configs;
    boost::split(configs, parameters_.config, boost::is_any_of(" "));
    for (const auto& config : configs) {
      std::ifstream inf(config);
      mjlib::base::system_error::throw_if(
          !inf.is_open(),
          fmt::format("could not open config file '{}'", parameters_.config));

      mjlib::base::Json5ReadArchive(inf).Accept(&config_);
    }

    context_.emplace(config_, &current_command_, &status_.state);

    PopulateStatusRequest();

    period_s_ = config_.period_s;
    rate_hz_ = static_cast<int>(1.0 / config_.period_s);
    timer_.start(mjlib::base::ConvertSecondsToDuration(period_s_),
                 std::bind(&Impl::HandleTimer, this, pl::_1));

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void Command(const HC& command) {
    const auto now = Now();
    const bool higher_priority = command.priority >= current_command_.priority;
    const bool stale =
        !current_command_timestamp_.is_not_a_date_time() &&
        (mjlib::base::ConvertDurationToSeconds(
            now - current_command_timestamp_) > parameters_.command_timeout_s);
    if (!higher_priority && !stale) {
      return;
    }

    CommandLog command_log;
    command_log.timestamp = now;
    command_log.command = &command;

    current_command_ = command;
    current_command_timestamp_ = now;

    // Update our logging status.
    if (command.log != HoverbotCommand::Log::kUnset) {
      if (command.log == HoverbotCommand::Log::kEnable &&
          !telemetry_log_->IsOpen()) {
        base::OpenMaybeTimestampedLog(
            telemetry_log_,
            parameters_.log_filename_base,
            base::kTimestamped);
      } else if (command.log == HoverbotCommand::Log::kDisable &&
                 telemetry_log_->IsOpen()) {
        telemetry_log_->Close();
      }
    }

    command_signal_(&command_log);
  }

  void PopulateStatusRequest() {
    status_request_ = {};

    for (const auto& joint : config_.joints) {
      status_request_.push_back({});
      auto& current = status_request_.back();
      current.id = joint.id;

      // Read mode, position, velocity, and torque.
      current.request.ReadMultiple(moteus::Register::kMode, 4, 1);
      current.request.ReadMultiple(moteus::Register::kVoltage, 3, 0);

      if (parameters_.servo_debug) {
        current.request.ReadMultiple(moteus::Register::kPositionKp, 5, 1);
      }
    }

    config_status_request_ = {};
    for (const auto& joint : config_.joints) {
      config_status_request_.push_back({});
      auto& current = config_status_request_.back();
      current.id = joint.id;

      // While configuring, we request a few more things.
      current.request.ReadMultiple(moteus::Register::kMode, 4, 1);
      current.request.ReadMultiple(moteus::Register::kRezeroState, 4, 0);
      current.request.ReadMultiple(moteus::Register::kRegisterMapVersion, 1, 2);
      current.request.ReadMultiple(moteus::Register::kSerialNumber, 3, 2);
    }
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    if (!pi3hat_) { return; }
    if (outstanding_) { return; }

    timing_ = ControlTiming(executor_, timing_.cycle_start());

    if (timing_.status().delta_s > 1.5 * period_s_) {
      // We likely skipped a cycle.  Warn.
      WarnRateLimited(fmt::format("Skipped cycle: delta_s={}",
                                  timing_.status().delta_s));
    }

    outstanding_ = true;

    status_reply_ = {};

    // Ask for the IMU and the servo data simultaneously.
    outstanding_status_requests_ = 0;

    auto* request = [&]() {
      if (status_.mode == HM::kConfiguring) {
        return &config_status_request_;
      }
      return &status_request_;
    }();
    pi3hat_->Cycle(&imu_data_, request, &status_reply_,
                   std::bind(&Impl::HandleStatus, this, pl::_1));
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    timing_.finish_query();

    imu_signal_(&imu_data_);

    // If we don't have all servos, then skip this cycle.
    const uint16_t servo_bitmask = [&]() {
      uint16_t result = 0;
      for (const auto& item : status_reply_) {
        result |= (1 << item.id);
      }
      return result;
    }();
    const int found_servos = [&]() {
      int result = 0;
      for (int i = 1; i <= kNumServos; i++) {
        if (servo_bitmask & (1 << i)) {
          result++;
        }
      }
      return result;
    }();
    status_.missing_replies = kNumServos - found_servos;

    if (found_servos != kNumServos) {
      if (status_.state.joints.size() != kNumServos) {
        // We have to get at least one full set before we can start
        // updating.
        std::string missing;
        for (int i = 1; i <= kNumServos; i++) {
          if ((servo_bitmask & (1 << i)) == 0) {
            if (!missing.empty()) { missing += ","; }
            missing += fmt::format("{}", i);
          }
        }
        const std::string message =
            fmt::format("Could not find one or more of "
                        "the following servos: {}", missing);
        log_.warn(message);
        status_.fault = message;

        outstanding_ = false;
        return;
      }
    }

    // Fill in the status structure.
    if (!UpdateStatus()) {
      // Guess we didn't have enough to actually do anything.
      outstanding_ = false;
      return;
    }

    timing_.finish_status();

    if (std::abs(imu_data_.euler_deg.roll) > 45 ||
        std::abs(imu_data_.euler_deg.pitch) > 45) {
      if (status_.mode != HM::kFault) {
        Fault("Tipping over");
      }
    }

    // Now run our control loop and generate our command.
    std::swap(control_log_, old_control_log_);
    *control_log_ = {};
    RunControl();

    timing_.finish_control();

    if (!client_command_.empty()) {
      client_command_reply_.clear();
      pi3hat_->AsyncTransmit(
          &client_command_, &client_command_reply_,
          std::bind(&Impl::HandleCommand, this, pl::_1));
    } else {
      HandleCommand({});
    }
  }

  void HandleCommand(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);
    outstanding_ = false;

    timing_.finish_command();
    status_.timestamp = Now();
    status_.timing = timing_.status();

    status_signal_(&status_);
  }

  std::optional<double> MaybeGetSign(int id) const {
    for (const auto& joint : config_.joints) {
      if (joint.id == id) { return joint.sign; }
    }
    return {};
  }

  bool UpdateStatus() {
    if (status_.mode == HM::kConfiguring) {
      // Try to update our config structure.
      UpdateConfiguringStatus();
    }

    auto find_or_make_joint = [&](int id) -> HoverbotState::Joint& {
      for (auto& joint : status_.state.joints) {
        if (joint.id == id) { return joint; }
      }
      status_.state.joints.push_back({});
      auto& result = status_.state.joints.back();
      result.id = id;
      return result;
    };

    for (const auto& reply : status_reply_) {
      const auto maybe_sign = MaybeGetSign(reply.id);
      if (!maybe_sign) {
        log_.warn(fmt::format("Reply from unknown servo {}", reply.id));
        return false;
      }

      HoverbotState::Joint& out_joint = find_or_make_joint(reply.id);

      out_joint.id = reply.id;

      const double sign = *maybe_sign;

      const auto* maybe_value = std::get_if<moteus::Value>(&reply.value);
      if (!maybe_value) { continue; }
      const auto& value = *maybe_value;
      switch (static_cast<moteus::Register>(reply.reg)) {
        case moteus::kMode: {
          out_joint.mode = moteus::ReadInt(value);
          break;
        }
        case moteus::kPosition: {
          out_joint.angle_deg = sign * moteus::ReadPosition(value);
          break;
        }
        case moteus::kVelocity: {
          out_joint.velocity_dps = sign * moteus::ReadVelocity(value);
          break;
        }
        case moteus::kTorque: {
          out_joint.torque_Nm = sign * moteus::ReadTorque(value);
          break;
        }
        case moteus::kVoltage: {
          out_joint.voltage = moteus::ReadVoltage(value);
          break;
        }
        case moteus::kTemperature: {
          out_joint.temperature_C = moteus::ReadTemperature(value);
          break;
        }
        case moteus::kFault: {
          out_joint.fault = moteus::ReadInt(value);
          break;
        }
        case moteus::kPositionKp: {
          out_joint.kp_Nm = sign * moteus::ReadTorque(value);
          break;
        }
        case moteus::kPositionKi: {
          out_joint.ki_Nm = sign * moteus::ReadTorque(value);
          break;
        }
        case moteus::kPositionKd: {
          out_joint.kd_Nm = sign * moteus::ReadTorque(value);
          break;
        }
        case moteus::kPositionFeedforward: {
          out_joint.feedforward_Nm = sign * moteus::ReadTorque(value);
          break;
        }
        case moteus::kPositionCommand: {
          out_joint.command_Nm = sign * moteus::ReadTorque(value);
          break;
        }
        default: {
          break;
        }
      }
    }

    std::sort(status_.state.joints.begin(), status_.state.joints.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.id < rhs.id;
              });

    if (status_.mode != HM::kFault) {
      std::string fault;

      for (const auto& joint : status_.state.joints) {
        if (joint.fault) {
          if (!fault.empty()) {
            fault += ", ";
          }
          fault += fmt::format("servo {} fault: {}", joint.id, joint.fault);
        }
      }
      if (!fault.empty()) {
        Fault(fault);
      }
    }

    // We should only be here if we have something for all our joints.
    if (status_.state.joints.size() != kNumServos) {
      return false;
    }

    {
      const double min_voltage =
          Min(status_.state.joints.begin(), status_.state.joints.end(),
              [](const auto& joint) { return joint.voltage; });
      auto& out_voltage = status_.state.robot.voltage;

      if (out_voltage == 0.0) {
        out_voltage = min_voltage;
      } else {
        const double alpha =
            std::pow(0.5, config_.period_s / config_.voltage_filter_s);
        out_voltage = alpha * out_voltage + (1.0 - alpha) * min_voltage;
      }

      if (out_voltage < config_.min_voltage) {
        Fault(fmt::format(
                  "Battery low: {} < {}", out_voltage, config_.min_voltage));
      }
    }

    return true;
  }

  void UpdateConfiguringStatus() {
    auto& reported = reported_servo_config_;

    auto find_or_make_servo = [&](int id) -> ReportedServoConfig::Servo& {
      for (auto& servo : reported.servos) {
        if (servo.id == id) { return servo; }
      }
      reported.servos.push_back({});
      auto& result = reported.servos.back();
      result.id = id;
      return result;
    };

    for (const auto& reply : status_reply_) {
      auto& out_servo = find_or_make_servo(reply.id);

      const auto* maybe_value = std::get_if<moteus::Value>(&reply.value);
      if (!maybe_value) { continue; }
      const auto& value = *maybe_value;

      switch (static_cast<moteus::Register>(reply.reg)) {
        case moteus::kRezeroState: {
          out_servo.rezero_state = moteus::ReadInt(value);
          break;
        }
        case moteus::kRegisterMapVersion: {
          out_servo.register_map_version = moteus::ReadInt(value);
          break;
        }
        case moteus::kSerialNumber1: {
          const auto sn = static_cast<uint32_t>(moteus::ReadInt(value));
          std::memcpy(&out_servo.serial_number[0], &sn, sizeof(sn));
          break;
        }
        case moteus::kSerialNumber2: {
          const auto sn = static_cast<uint32_t>(moteus::ReadInt(value));
          std::memcpy(&out_servo.serial_number[4], &sn, sizeof(sn));
          break;
        }
        case moteus::kSerialNumber3: {
          const auto sn = static_cast<uint32_t>(moteus::ReadInt(value));
          std::memcpy(&out_servo.serial_number[8], &sn, sizeof(sn));
          break;
        }
        default: {
          break;
        }
      }
    }

    reported.timestamp = Now();
    servo_config_signal_(&reported);
  }

  void RunControl() {
    if (current_command_.mode != status_.mode) {
      MaybeChangeMode();
    }

    switch (status_.mode) {
      case HM::kConfiguring: {
        DoControl_Configuring();
        break;
      }
      case HM::kStopped: {
        DoControl_Stopped();
        break;
      }
      case HM::kFault: {
        DoControl_Fault();
        break;
      }
      case HM::kZeroVelocity: {
        DoControl_ZeroVelocity();
        break;
      }
      case HM::kJoint: {
        DoControl_Joint();
        break;
      }
      case HM::kPitch: {
        DoControl_Pitch();
        break;
      }
      case HM::kDrive: {
        DoControl_Drive();
        break;
      }
      case HM::kNumModes: {
        mjlib::base::AssertNotReached();
      }
    }
  }

  void MaybeChangeMode() {
    const auto old_mode = status_.mode;
    if (status_.mode == HM::kConfiguring) {
      // We can only leave this mode after we have heard from all the
      // servos, they are all rezeroed, and have an appropriate
      // register map version.
      if (!IsConfiguringDone()) { return; }
    }

    switch (current_command_.mode) {
      case HM::kConfiguring:
      case HM::kNumModes:
      case HM::kFault: {
        mjlib::base::AssertNotReached();
      }
      case HM::kStopped: {
        // It is always valid (although I suppose not always a good
        // idea) to enter the stopped mode.
        status_.mode = HM::kStopped;
        break;
      }
      case HM::kZeroVelocity:
      case HM::kJoint: {
        // We can always do these if not faulted.
        if (status_.mode == HM::kFault) { return; }
        status_.mode = current_command_.mode;
        break;
      }
      case HM::kPitch:
      case HM::kDrive: {
        // TODO!  Optionally enforce a "stand up" phase.
        status_.mode = current_command_.mode;
        break;
      }
    }

    if (status_.mode != old_mode) {
      log_.warn(fmt::format("Changed mode from {} to {}",
                            old_mode, status_.mode));
      switch (old_mode) {
        case HM::kConfiguring:
        case HM::kStopped:
        case HM::kFault:
        case HM::kZeroVelocity:
        case HM::kJoint: {
          status_.state.pitch.pitch_pid.Clear();
          status_.state.pitch.yaw_pid.Clear();
          status_.state.pitch.yaw_target = imu_data_.euler_deg.yaw;
          break;
        }
        case HM::kPitch:
        case HM::kDrive:
        case HM::kNumModes: {
          break;
        }
      }
      status_.mode_start = Now();
    }
  }

  bool IsConfiguringDone() {
    // We must have heard from all servos.
    if (reported_servo_config_.servos.size() != kNumServos) {
      status_.fault = "missing servos";
      return false;
    }

    // All of them must have been rezerod and have the current
    // register map.
    for (const auto& servo : reported_servo_config_.servos) {
      if (servo.register_map_version != moteus::kCurrentRegisterMapVersion) {
        status_.fault = fmt::format("servo {} has incorrect register version {}",
                                    servo.id, servo.register_map_version);
        return false;
      }
    }

    status_.fault = "";
    return true;
  }

  void DoControl_Configuring() {
    // If we are configuring, and have received a status that *all* of
    // our servos are not zeroed, then we skip a control cycle and
    // instead rezero them.
    EmitStop();
  }

  void DoControl_Stopped() {
    status_.fault = "";

    EmitStop();
  }

  void EmitStop() {
    std::vector<HC::Joint> out_joints;
    for (const auto& joint : config_.joints) {
      HC::Joint out_joint;
      out_joint.id = joint.id;
      out_joint.power = false;
      out_joints.push_back(out_joint);
    }

    ControlJoints(std::move(out_joints));
  }

  void Fault(std::string_view message) {
    status_.mode = HM::kFault;
    status_.fault = message;
    status_.mode_start = Now();

    log_.warn("Fault: " + std::string(message));

    DoControl_Fault();
  }

  void DoControl_Fault() {
    DoControl_ZeroVelocity();
  }

  void DoControl_ZeroVelocity() {
    std::vector<HC::Joint> out_joints;
    for (const auto& joint : config_.joints) {
      HC::Joint out_joint;
      out_joint.id = joint.id;
      out_joint.power = true;
      out_joint.zero_velocity = true;
      out_joints.push_back(out_joint);
    }

    ControlJoints(std::move(out_joints));
  }

  void DoControl_Joint() {
    ControlJoints(current_command_.joints);
  }

  void ControlPitch(const HC::Pitch& pitch) {
    control_log_->pitch = pitch;

    const auto pitch_torque_Nm =
        pitch_pid_.Apply(imu_data_.euler_deg.pitch, pitch.pitch_deg,
                         imu_data_.rate_dps.y(), pitch.pitch_rate_dps,
                         rate_hz_);

    status_.state.pitch.yaw_target += pitch.yaw_rate_dps * period_s_;
    const auto yaw_torque_Nm =
        yaw_pid_.Apply(imu_data_.euler_deg.yaw, status_.state.pitch.yaw_target,
                       imu_data_.rate_dps.z(), pitch.yaw_rate_dps,
                       rate_hz_);

    control_log_->pitch_torque_Nm = pitch_torque_Nm;
    control_log_->yaw_torque_Nm = yaw_torque_Nm;

    std::vector<HC::Joint> joints;
    for (int id : {1, 2}) {
      HC::Joint joint;
      joint.id = id;
      joint.power = true;
      const double yaw_sign = id == 1 ? -1 : 1;
      joint.torque_Nm = pitch_torque_Nm + yaw_sign * yaw_torque_Nm;
      joint.kp_scale = 0.0;
      joint.kd_scale = 0.0;
      joints.push_back(joint);
    }

    ControlJoints(std::move(joints));
  }

  void DoControl_Pitch() {
    ControlPitch(current_command_.pitch);
  }

  void ControlDrive(const HC::Drive& drive) {
    HC::Pitch pitch;
    ControlPitch(pitch);
  }

  void DoControl_Drive() {
    ControlDrive(current_command_.drive);
  }

  void ControlJoints(std::vector<HC::Joint> joints) {
    control_log_->joints = std::move(joints);
    std::sort(control_log_->joints.begin(),
              control_log_->joints.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.id < rhs.id;
              });

    EmitControl();
  }

  void EmitControl() {
    control_log_->timestamp = Now();
    control_signal_(control_log_);

    size_t pos = 0;
    for (const auto& joint : control_log_->joints) {
      if (client_command_.size() <= pos) {
        client_command_.resize(client_command_.size() + 1);
      }

      auto& request = client_command_[pos++];
      request.request.clear();
      request.id = joint.id;

      constexpr double kInf = std::numeric_limits<double>::infinity();
      std::optional<double> max_torque_Nm =
          (parameters_.max_torque_Nm >= 0.0 || !!joint.max_torque_Nm) ?
          std::min(parameters_.max_torque_Nm < 0.0 ?
                   kInf : parameters_.max_torque_Nm,
                   joint.max_torque_Nm.value_or(kInf)) :
          std::optional<double>();

      const auto mode = [&]() {
        if (joint.power == false) {
          return moteus::Mode::kStopped;
        } else if (joint.zero_velocity) {
          return moteus::Mode::kZeroVelocity;
        } else {
          return moteus::Mode::kPosition;
        }
      }();

      request.request.WriteSingle(moteus::kMode, static_cast<int8_t>(mode));

      auto& values = values_cache_;
      values.resize(0);

      if (mode == moteus::Mode::kPosition ||
          mode == moteus::Mode::kZeroVelocity) {
        const auto maybe_sign = MaybeGetSign(joint.id);
        if (!maybe_sign) {
          log_.warn(fmt::format("Unknown servo {}", joint.id));
          continue;
        }
        const double sign = *maybe_sign;

        if (joint.angle_deg != 0.0) { values.resize(1); }
        if (joint.velocity_dps != 0.0) { values.resize(2); }
        if (joint.torque_Nm != 0.0) { values.resize(3); }
        if (max_torque_Nm) { values.resize(6); }
        if (joint.stop_angle_deg) { values.resize(7); }

        for (size_t i = 0; i < values.size(); i++) {
          switch (i) {
            case 0: {
              values[i] = moteus::WritePosition(
                  sign * joint.angle_deg, moteus::kInt16);
              break;
            }
            case 1: {
              values[i] = moteus::WriteVelocity(
                  sign * joint.velocity_dps, moteus::kInt16);
              break;
            }
            case 2: {
              values[i] = moteus::WriteTorque(
                  sign * joint.torque_Nm, moteus::kInt16);
              break;
            }
            case 3:
            case 4: {
              values[i] = moteus::WritePwm(1.0, moteus::kInt16);
              break;
            }
            case 5: {
              values[i] = moteus::WriteTorque(
                  max_torque_Nm.value_or(kInf),
                  moteus::kInt16);
              break;
            }
            case 6: {
              values[i] = moteus::WritePosition(
                  sign * joint.stop_angle_deg.value_or(
                      std::numeric_limits<double>::quiet_NaN()),
                  moteus::kInt16);
              break;
            }

          }
        }

        if (!values.empty()) {
          request.request.WriteMultiple(moteus::kCommandPosition, values);
        }

        // We do kp and kd separately so we can use the float type.
        values.clear();
        if (joint.kp_scale) { values.resize(1); }
        if (joint.kd_scale) { values.resize(2); }
        for (size_t i = 0; i < values.size(); i++) {
          switch (i) {
            case 0: {
              double kp = joint.kp_scale.value_or(1.0);
              if (kp < 0.0) {
                kp = 0.0;
                log_.warn("negative joint kp!");
              }
              values[i] = moteus::WritePwm(kp, moteus::kFloat);
              break;
            }
            case 1: {
              double kd = joint.kd_scale.value_or(1.0);
              if (kd < 0.0) {
                kd = 0.0;
                log_.warn("negative joint kd!");
              }
              values[i] = moteus::WritePwm(kd, moteus::kFloat);
              break;
            }
          }
        }

        if (!values.empty()) {
          request.request.WriteMultiple(moteus::kCommandKpScale, values);
        }
      }
    }
    if (client_command_.size() > pos) {
      client_command_.resize(pos);
    }
  }

  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  void WarnRateLimited(const std::string& message) {
    const boost::posix_time::time_duration kRateLimitTime =
        boost::posix_time::seconds(1);

    const auto now = Now();
    if (last_warn_timestamp_.is_not_a_date_time() ||
        (now - last_warn_timestamp_) > kRateLimitTime) {
      last_warn_timestamp_ = now;
      log_.warn(message);
    }
  }

  boost::asio::any_io_executor executor_;
  mjlib::telemetry::FileWriter* const telemetry_log_;
  Parameters parameters_;

  base::LogRef log_ = base::GetLogInstance("HoverbotControl");

  Config config_;
  std::optional<HoverbotContext> context_;

  HoverbotControl::Status status_;
  HC current_command_;
  boost::posix_time::ptime current_command_timestamp_;
  ReportedServoConfig reported_servo_config_;

  std::array<ControlLog, 2> control_logs_;
  ControlLog* control_log_ = &control_logs_[0];
  ControlLog* old_control_log_ = &control_logs_[1];

  double period_s_ = 0.0;
  int rate_hz_ = 1;
  mjlib::io::RepeatingTimer timer_;
  using Client = mjlib::multiplex::AsioClient;

  Pi3hatGetter pi3hat_getter_;

  Pi3hatInterface* pi3hat_ = nullptr;

  using Request = Client::Request;
  Request status_request_;
  Request config_status_request_;
  Client::Reply status_reply_;

  Request client_command_;
  Client::Reply client_command_reply_;

  bool outstanding_ = false;
  ControlTiming timing_{executor_, {}};

  int outstanding_status_requests_ = 0;
  AttitudeData imu_data_;

  boost::signals2::signal<void (const Status*)> status_signal_;
  boost::signals2::signal<void (const CommandLog*)> command_signal_;
  boost::signals2::signal<void (const ControlLog*)> control_signal_;
  boost::signals2::signal<void (const AttitudeData*)> imu_signal_;
  boost::signals2::signal<
    void (const ReportedServoConfig*)> servo_config_signal_;

  std::vector<moteus::Value> values_cache_;

  boost::posix_time::ptime last_warn_timestamp_;


  mjlib::base::PID pitch_pid_{
    &config_.pitch.pitch_pid, &status_.state.pitch.pitch_pid};
  mjlib::base::PID yaw_pid_{
    &config_.pitch.yaw_pid, &status_.state.pitch.yaw_pid};


};

HoverbotControl::HoverbotControl(base::Context& context,
                                   Pi3hatGetter pi3hat_getter)
    : impl_(std::make_unique<Impl>(context, pi3hat_getter)) {}

HoverbotControl::~HoverbotControl() {}

void HoverbotControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void HoverbotControl::Command(const HC& command) {
  impl_->Command(command);
}

const HoverbotControl::Status& HoverbotControl::status() const {
  return impl_->status_;
}

clipp::group HoverbotControl::program_options() {
  return mjlib::base::ClippArchive().Accept(&impl_->parameters_).release();
}

}
}
