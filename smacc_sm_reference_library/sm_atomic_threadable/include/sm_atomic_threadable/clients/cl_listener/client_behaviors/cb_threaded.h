#pragma once

#include <smacc/smacc.h>
#include <smacc/smacc_client_behavior_base.h>
#include <smacc/smacc_threadable.h>

namespace sm_atomic_threadable {

class CbThreaded : public smacc::ISmaccThreadable,
                   public smacc::ISmaccClientBehavior {
 public:
  CbThreaded() : ISmaccThreadable() {}

  void onEntry() override {
    ClListener<1>* cl_listener1;
    this->requiresClient(cl_listener1);
    cl_listener1->onMessageReceived(&CbThreaded::onMessageReceived1, this);

    ClListener<2>* cl_listener2;
    this->requiresClient(cl_listener2);
    cl_listener2->onMessageReceived(&CbThreaded::onMessageReceived2, this);
  }

  void onExit() override {
    ROS_INFO("Executing CbThread onExit, waiting for threads to stop...");
    this->stop();
    ROS_INFO("Exiting");
  }

 private:
  void onMessageReceived1(const std_msgs::UInt16& msg) {
    // simulate a slow process, 2.5Hz (400ms period)
    this->runOnThread([]() {
      std::this_thread::sleep_for(std::chrono::milliseconds{400});
      ROS_INFO("LOW frequency out");
    });
  }

  void onMessageReceived2(const std_msgs::UInt16& msg) {
    ROS_INFO("HIGH frequency out");
  }
};
}  // namespace sm_atomic_threadable
