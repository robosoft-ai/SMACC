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

  boost::signals2::scoped_connection c1_;
  boost::signals2::scoped_connection c2_;
  boost::signals2::scoped_connection c3_;

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
      c1_ = sensor_->onMessageReceived.connect(
          [this](auto &msg) {
            auto *ev2 = new EvTopicMessage<TDerived, TObjectTag>();
            this->postEvent(ev2);
          });

      c2_ = sensor_->onFirstMessageReceived.connect(
          [this](auto &msg) {
            auto event = new EvTopicInitialMessage<TDerived, TObjectTag>();
            this->postEvent(event);
          });

      c3_ = sensor_->onMessageTimeout.connect(
          [this](auto &msg) {
            auto event = new EvTopicMessageTimeout<TDerived, TObjectTag>();
            this->postEvent(event);
          });
    };
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
