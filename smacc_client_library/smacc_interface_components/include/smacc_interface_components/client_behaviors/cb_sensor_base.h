#pragma once

#include <smacc/smacc_client_behavior.h>
#include <smacc_interface_components/clients/sensor_client.h>

namespace smacc
{
template <typename ClientType>
class SensorTopic : public smacc::SmaccClientBehavior
{
public:
  typedef typename ClientType::TMessageType TMessageType;

  ClientType *sensor_;

  SensorTopic()
  {
    sensor_ = nullptr;
  }

  static std::string getEventLabel()
  {
    // show ros message type
    return demangleSymbol(typeid(TMessageType).name());
  }

  std::function<void()> deferedEventPropagation;

  template <typename TObjectTag, typename TDerived>
  void configureEventSourceTypes()
  {
    deferedEventPropagation = [=]() {
      // just propagate the client events from this client behavior source.
      sensor_->onMessageReceived(&SensorTopic<ClientType>::propagateEvent<EvTopicMessage<TDerived, TObjectTag>>, this);
      sensor_->onFirstMessageReceived(&SensorTopic<ClientType>::propagateEvent<EvTopicInitialMessage<TDerived, TObjectTag>>, this);
      sensor_->onMessageTimeout(&SensorTopic<ClientType>::propagateEvent2<EvTopicMessageTimeout<TDerived, TObjectTag>>, this);
    };
  }

  template <typename EvType>
  void propagateEvent(const TMessageType &msg)
  {
    this->postEvent<EvType>();
  }

  template <typename EvType>
  void propagateEvent2(const ros::TimerEvent &tdata)
  {
    this->postEvent<EvType>();
  }

  virtual void onEntry() override
  {
    ROS_INFO("SensorTopic onEntry. Requires client of type '%s'", demangleSymbol<ClientType>().c_str());

    this->requiresClient(sensor_);

    if (sensor_ == nullptr)
    {
      ROS_FATAL_STREAM("Sensor client behavior needs a client of type: " << demangleSymbol<ClientType>() << " but it is not found.");
    }
    else
    {
      deferedEventPropagation();
      ROS_INFO("SensorTopic onEntry. sensor initialize");
      sensor_->initialize();
    }
  }

  void onExit()
  {
  }

  virtual void onMessageCallback(const TMessageType &msg)
  {
    // empty to fill by sensor customization based on inheritance
  }
};
} // namespace smacc
