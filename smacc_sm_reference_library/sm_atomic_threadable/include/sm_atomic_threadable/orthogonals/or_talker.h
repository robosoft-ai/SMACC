#pragma once

#include <smacc/smacc.h>

namespace sm_atomic_threadable {
class OrTalker : public smacc::Orthogonal<OrTalker> {
 public:
  void onInitialize() override {
    // simulate 15Hz talkers
    auto cl_talker1 = this->createClient<ClTalker<1>>("/tick_1");
    cl_talker1->initialize();

    auto cl_talker2 = this->createClient<ClTalker<2>>("/tick_2");
    cl_talker2->initialize();
  }
};
}  // namespace sm_atomic_threadable
