#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <smacc/smacc_substate_behavior.h>


namespace smacc
{
template <typename MessageType>
class SensorTopic : public smacc::SmaccSubStateBehavior
{
public:
  typedef MessageType TMessageType;

  SensorClient<MessageType> *sensor_;

  ros::Duration timeoutDuration_;
  std::string topicName_;
  int queueSize_;
  boost::signals2::scoped_connection c1_;
  boost::signals2::scoped_connection c2_;
  boost::signals2::scoped_connection c3_;

  SensorTopic(std::string topicName, int queueSize = 1, ros::Duration timeout = ros::Duration(5))
  {
    timeoutDuration_ = timeout;
    queueSize_ = queueSize;
    topicName_ = topicName;
  }

  void onEntry()
  {
    this->requiresComponent(sensor_);

    c1_ = sensor_->onMessageReceived.connect(
        [this](auto &msg) {
          auto *ev2 = new EvTopicMessage<SensorTopic<MessageType>>();
          this->postEvent(ev2);
        });

    c2_ = sensor_->onFirstMessageReceived.connect(
        [this](auto &msg) {
          auto event = new EvTopicInitialMessage<SensorTopic<MessageType>>();
          this->postEvent(event);
        });

    c3_ = sensor_->onMessageTimeout.connect(
        [this](auto &msg) {
          auto event = new EvTopicMessageTimeout<SensorTopic<MessageType>>();
          this->postEvent(event);
        });

    sensor_->initialize(topicName_, queueSize_, timeoutDuration_);
  }

  bool onExit()
  {
  }

  virtual void onMessageCallback(const MessageType &msg)
  {

    // empty to fill by sensor customization based on inheritance
  }
};
} // namespace smacc
