#pragma once

#include <smacc/smacc.h>

namespace sm_atomic_threadable {
class OrTimer : public smacc::Orthogonal<OrTimer> {
 public:
  void onInitialize() override {
    // simulate 15Hz talker
    auto timer_client =
        this->createClient<cl_ros_timer::ClRosTimer>(ros::Duration{1.0 / 15.0});
    timer_client->initialize();
  }
};
}  // namespace sm_atomic_threadable
