#pragma once

#include <smacc/smacc.h>
#include <smacc/smacc_client_behavior_base.h>
#include <smacc/smacc_threadable.h>

namespace sm_atomic_threadable {

template <typename TSource, typename TOrthogonal>
struct EvExitCb : sc::event<EvExitCb<TSource, TOrthogonal>> {};

class CbThread : public smacc::ISmaccThreadable,
                 public smacc::ISmaccClientBehavior {
 public:
  CbThread() : ISmaccThreadable() {}

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation() {
    postExitEvent = [this]() {
      this->postEvent<EvExitCb<TSourceObject, TOrthogonal>>();
    };
  }

  void onEntry() override {
    ROS_INFO("Executing CbThread onEntry()");
    this->runOnThread([]() {
      ROS_INFO("Executing worker thread, about to sleep for 2s...");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      ROS_INFO("Thread waking up");
    });

    ROS_INFO("Exiting CbThread onEntry()");
    postExitEvent();
    ROS_INFO("Exiting CbThread onEntry()");
  }

  void onExit() override {
    ROS_INFO("Executing CbThread onExit, waiting for threads to stop...");
    this->stop();
    ROS_INFO("Exiting");
  }

 private:
  std::function<void()> postExitEvent;
};
}  // namespace sm_atomic_threadable
