#pragma once

#include <smacc/smacc_substate_behavior.h>

namespace smacc
{
template <typename Derived, typename MessageType, typename ClientType>
class SensorTopic : public smacc::SmaccSubStateBehavior
{
public:
  typedef MessageType TMessageType;

  ClientType *sensor_;

  boost::signals2::scoped_connection c1_;
  boost::signals2::scoped_connection c2_;
  boost::signals2::scoped_connection c3_;

  SensorTopic()
  {
    sensor_=nullptr;
  }

  static std::string getEventLabel()
  {
    // show ros message type
    return demangleSymbol(typeid(MessageType).name());
  }

  virtual void onEntry() override
  {
    ROS_INFO("SensorTopic onEntry. Requires client:");

    this->requiresClient(sensor_);

    if(sensor_==nullptr)
    {
      ROS_FATAL_STREAM("Sensor Substate behavior needs a client of type: "<< demangleSymbol<ClientType>()<< " but it is not found.");
    }
    else
    {
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

      ROS_INFO("SensorTopic onEntry. sensor initialize");
      sensor_->initialize();
    }
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
