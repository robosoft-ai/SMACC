#pragma once

#include <smacc/smacc.h>
#include <std_msgs/UInt16.h>

#include <chrono>
#include <thread>

namespace sm_atomic_threadable {
class CbUnthreaded : public smacc::SmaccClientBehavior {
 public:
  void onEntry() override {
    ClListener<1>* cl_listener1;
    this->requiresClient(cl_listener1);
    cl_listener1->onMessageReceived(&CbUnthreaded::onMessageReceived1, this);

    ClListener<2>* cl_listener2;
    this->requiresClient(cl_listener2);
    cl_listener2->onMessageReceived(&CbUnthreaded::onMessageReceived2, this);
  }

 private:
  void onMessageReceived1(const std_msgs::UInt16& msg) {
    // simulate a slow process, 2.5Hz (400ms period)
    std::this_thread::sleep_for(std::chrono::milliseconds{400});
    ROS_INFO("LOW frequency out");
  }

  void onMessageReceived2(const std_msgs::UInt16& msg) {
    ROS_INFO("HIGH frequency out");
  }
};
}  // namespace sm_atomic_threadable
