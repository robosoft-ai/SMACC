#pragma once

#include <smacc/smacc.h>

namespace sm_atomic_threadable {
class OrKeyboard : public smacc::Orthogonal<OrKeyboard> {
 public:
  void onInitialize() override {
    auto client = this->createClient<cl_keyboard::ClKeyboard>();
    client->initialize();
  }
};
}  // namespace sm_atomic_threadable
