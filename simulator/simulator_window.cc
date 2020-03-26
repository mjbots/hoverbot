// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

#include "simulator_window.h"

#include <boost/filesystem.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/gui/LoadGlut.hpp>

#include "mjlib/base/fail.h"
#include "mjlib/base/json5_read_archive.h"
#include "mjlib/io/now.h"
#include "mjlib/io/debug_deadline_service.h"

#include "base/common.h"
#include "base/context_full.h"

#include "mech/quadruped.h"

#include "simulator/make_robot.h"

namespace dd = dart::dynamics;
namespace ds = dart::simulation;

namespace mjmech {
namespace simulator {

namespace {
class SimMultiplex : public mjlib::multiplex::AsioClient {
 public:
  struct Options {
    template <typename Archive>
    void Serialize(Archive*) {}
  };

  SimMultiplex(boost::asio::executor executor, const Options&)
      : executor_(executor) {}
  ~SimMultiplex() override {}

  void AsyncRegister(
      const IdRequest&, SingleReply* reply,
      mjlib::io::ErrorCallback callback) override {
    *reply = {};
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void AsyncRegisterMultiple(
      const std::vector<IdRequest>&, Reply* reply,
      mjlib::io::ErrorCallback callback) override {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id, uint32_t channel, const TunnelOptions& options) override {
    return {};
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

 private:
  boost::asio::executor executor_;
};

class SimImu : public mech::ImuClient {
 public:
  struct Options {
    template <typename Archive>
    void Serialize(Archive*) {}
  };

  SimImu(boost::asio::executor executor, const Options&)
      : executor_(executor) {}

  ~SimImu() override {}

  void set_frame(dd::Frame* frame) {
    frame_ = frame;
  }

  void ReadImu(mech::AttitudeData* attitude,
               mjlib::io::ErrorCallback callback) override {
    if (frame_) {
      attitude->timestamp = mjlib::io::Now(executor_.context());

      // TODO(jpieper): Confirm the sign and magnitude of all these
      // things compared to the real thing.

      const Eigen::Isometry3d tf = frame_->getTransform();
      attitude->attitude = Sophus::SE3d(tf.matrix()).unit_quaternion();
      attitude->euler_deg = (180.0 / M_PI) * attitude->attitude.euler_rad();

      const Eigen::Vector3d rate_rps = frame_->getAngularVelocity();
      attitude->rate_dps.x() = base::Degrees(rate_rps[0]);
      attitude->rate_dps.y() = base::Degrees(rate_rps[1]);
      attitude->rate_dps.z() = -base::Degrees(rate_rps[2]);

      Eigen::Vector3d accel =
          frame_->getLinearAcceleration(Eigen::Vector3d(0, 0, 0));
      attitude->accel_mps2 = accel;
    } else {
      *attitude = {};
    }
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void set_data(const mech::AttitudeData& data) {
    data_ = data;
  }

 private:
  boost::asio::executor executor_;
  mech::AttitudeData data_;
  dd::Frame* frame_ = nullptr;
};
}

struct Options {
  bool start_disabled = false;
  std::string config = "configs/quada1.cfg";

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(start_disabled));
    a->Visit(MJ_NVP(config));
  }
};

class SimulatorWindow::Impl : public dart::gui::glut::SimWindow {
 public:
  static Impl* g_impl_;

  Impl(base::Context& context)
      : context_(context.context),
        executor_(context.executor),
        quadruped_(context) {
    g_impl_ = this;

    quadruped_.m()->multiplex_client->Register<SimMultiplex>("sim");
    quadruped_.m()->multiplex_client->set_default("sim");

    quadruped_.m()->imu_client->Register<SimImu>("sim");
    quadruped_.m()->imu_client->set_default("sim");

    floor_ = MakeFloor();
    mech::QuadrupedConfig config;
    {
      std::ifstream inf(options_.config);
      mjlib::base::system_error::throw_if(
          !inf.is_open(),
          fmt::format("could not open config file '{}'", options_.config));
      mjlib::base::Json5ReadArchive(inf).Accept(&config);
    }

    robot_ = MakeRobot(config);

    world_->addSkeleton(floor_);
    world_->addSkeleton(robot_);

    setWorld(world_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    // The GLUT timer will actually process our event loop, so it
    // needs to be going right away.
    StartGlutTimer();

    if (!options_.start_disabled) {
      // Send a space bar to get us simulating.
      SimWindow::keyboard(' ', 0, 0);
    }

    quadruped_.AsyncStart([this, callback=std::move(callback)](
                              const auto& ec) mutable {
        this->HandleStart(ec, std::move(callback));
      });
  }

  void HandleStart(const mjlib::base::error_code& ec,
                   mjlib::io::ErrorCallback callback) {
    if (ec) {
      boost::asio::post(
          executor_,
          std::bind(std::move(callback), ec));
      return;
    }

    imu_ = dynamic_cast<SimImu*>(quadruped_.m()->imu_client->selected());
    BOOST_ASSERT(imu_);
    multiplex_ = dynamic_cast<SimMultiplex*>(
        quadruped_.m()->multiplex_client->selected());
    BOOST_ASSERT(multiplex_);

    imu_->set_frame(robot_->getBodyNode("robot"));

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), ec));
  }

  static void GlobalHandleGlutTimer(int) {
    g_impl_->HandleGlutTimer();
  }

  void StartGlutTimer() {
    glutTimerFunc(1, &GlobalHandleGlutTimer, 0);
  }

  void HandleGlutTimer() {
    context_.poll();
    context_.reset();
    StartGlutTimer();
  }

  boost::asio::io_context& context_;
  boost::asio::executor executor_;

  Options options_;

  dd::SkeletonPtr floor_;
  dd::SkeletonPtr robot_;

  ds::WorldPtr world_ = std::make_shared<ds::World>();

  mech::Quadruped quadruped_;

  SimImu* imu_ = nullptr;
  SimMultiplex* multiplex_ = nullptr;
};

SimulatorWindow::Impl* SimulatorWindow::Impl::g_impl_ = nullptr;

SimulatorWindow::SimulatorWindow(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

SimulatorWindow::~SimulatorWindow() {}

clipp::group SimulatorWindow::program_options() {
  return clipp::group(
      mjlib::base::ClippArchive().Accept(&impl_->options_).release(),
      impl_->quadruped_.program_options());
}

void SimulatorWindow::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void SimulatorWindow::InitWindow(int x, int y, const char* name) {
  impl_->initWindow(x, y, name);
}

}
}
