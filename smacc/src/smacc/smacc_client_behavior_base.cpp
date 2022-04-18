#include <smacc/smacc_client_behavior.h>

namespace smacc
{
ISmaccClientBehavior::ISmaccClientBehavior()
{
  stateMachine_ = nullptr;
  currentState = nullptr;
  initialized = false;
}

ISmaccClientBehavior::~ISmaccClientBehavior()
{
  ROS_WARN("Client behavior deallocated.");
}

std::string ISmaccClientBehavior::getName() const
{
  return demangleSymbol(typeid(*this).name());
}

void ISmaccClientBehavior::runtimeConfigure()
{
  ROS_DEBUG("[%s] Default empty SmaccClientBehavior runtimeConfigure", this->getName().c_str());
}

void ISmaccClientBehavior::onEntry(){};

void ISmaccClientBehavior::onExit(){};

ros::NodeHandle ISmaccClientBehavior::getNode()
{
  return this->stateMachine_->getNode();
}

void ISmaccClientBehavior::executeOnEntry()
{
  initialized = false;
  ROS_DEBUG("[%s] Default empty SmaccClientBehavior onEntry", this->getName().c_str());
  this->onEntry();
  initialized = true;
}

void ISmaccClientBehavior::executeOnExit()
{
  ROS_DEBUG("[%s] Default empty SmaccClientBehavior onExit", this->getName().c_str());
  this->onExit();
  initialized = false;
}

void ISmaccClientBehavior::dispose()
{
}

bool ISmaccClientBehavior::isInitialized()
{
  return initialized;
}

}  // namespace smacc