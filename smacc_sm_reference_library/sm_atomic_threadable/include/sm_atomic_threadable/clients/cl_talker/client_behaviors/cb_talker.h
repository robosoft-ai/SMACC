#pragma once

#include <smacc/smacc.h>
#include <std_msgs/UInt16.h>

namespace sm_atomic_threadable {
template <int TIndex>
class CbTalker : public smacc::SmaccClientBehavior {
 public:
  void onEntry() override {
    cl_ros_timer::ClRosTimer* cl_timer;
    this->requiresClient(cl_timer);
    cl_timer->onTimerTick(&CbTalker::onTimerTick, this);

    this->requiresClient(cl_talker_);
  }

 private:
  void onTimerTick() {
    if (count == 65534) count = 0;

    std_msgs::UInt16 msg;
    msg.data = count++;
    cl_talker_->publish(msg);
  }

  ClTalker<TIndex>* cl_talker_;

  uint16_t count = 0;
};
}  // namespace sm_atomic_threadable
