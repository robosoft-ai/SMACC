#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <smacc/smacc_substate_behavior.h>


namespace smacc
{
template <typename Derived, typename MessageType>
class SensorTopic : public smacc::SmaccSubStateBehavior
{
public:
  typedef MessageType TMessageType;

  SensorClient<MessageType> *sensor_;

  boost::signals2::scoped_connection c1_;
  boost::signals2::scoped_connection c2_;
  boost::signals2::scoped_connection c3_;

  SensorTopic()
  {
  }

  void onEntry()
  {
    this->requiresClient(sensor_);

    c1_ = sensor_->onMessageReceived.connect(
        [this](auto &msg) {
          auto *ev2 = new EvTopicMessage<Derived>();
          this->postEvent(ev2);
        });

    c2_ = sensor_->onFirstMessageReceived.connect(
        [this](auto &msg) {
          auto event = new EvTopicInitialMessage<Derived>();
          this->postEvent(event);
        });

    c3_ = sensor_->onMessageTimeout.connect(
        [this](auto &msg) {
          auto event = new EvTopicMessageTimeout<Derived>();
          this->postEvent(event);
        });

    sensor_->initialize();
  }

  void onExit()
  {
  }

  virtual void onMessageCallback(const MessageType &msg)
  {

    // empty to fill by sensor customization based on inheritance
  }
};
} // namespace smacc
