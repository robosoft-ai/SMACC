#pragma once

#include <smacc/smacc.h>
#include <std_msgs/UInt16.h>

namespace sm_atomic_threadable {
class OrListener : public smacc::Orthogonal<OrListener> {
 public:
  void onInitialize() override {
    auto listener1 = this->createClient<ClListener<1>>("/tick_1");
    listener1->initialize();

    auto listener2 = this->createClient<ClListener<2>>("/tick_2");
    listener2->initialize();
  }
};
}  // namespace sm_atomic_threadable
