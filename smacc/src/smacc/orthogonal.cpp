#include <smacc/impl/smacc_state_machine_impl.h>
#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_orthogonal.h>

namespace smacc
{
void ISmaccOrthogonal::setStateMachine(ISmaccStateMachine *value)
{
  this->stateMachine_ = value;
  this->onInitialize();
}

void ISmaccOrthogonal::addClientBehavior(std::shared_ptr<smacc::SmaccClientBehavior> clBehavior)
{
  if (clBehavior != nullptr)
  {
    ROS_INFO("Setting Ortho %s client behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());
    clBehavior->stateMachine_ = this->getStateMachine();
    clBehavior->currentOrthogonal = this;

    clientBehaviors_.push_back(clBehavior);
  }
  else
  {
    ROS_INFO("Not behavioral State by orthogonal");
  }
}

void ISmaccOrthogonal::onInitialize()
{
}

std::string ISmaccOrthogonal::getName() const
{
  return demangleSymbol(typeid(*this).name());
}

void ISmaccOrthogonal::runtimeConfigure()
{
  for (auto &clBehavior : clientBehaviors_)
  {
    ROS_INFO("Orthogonal %s runtimeConfigure, current Behavior: %s", this->getName().c_str(),
             clBehavior->getName().c_str());

    clBehavior->runtimeConfigure();
  }
}

void ISmaccOrthogonal::assignClientToOrthogonal(smacc::ISmaccClient *client)
{
  client->setStateMachine(getStateMachine());
  client->setOrthogonal(this);
}

void ISmaccOrthogonal::onEntry()
{
  if (clientBehaviors_.size() > 0)
  {
    for (auto &clBehavior : clientBehaviors_)
    {
      auto cbname = clBehavior->getName().c_str();
      ROS_INFO("Orthogonal %s OnEntry, current Behavior: %s", this->getName().c_str(), cbname);

      try
      {
        clBehavior->onEntry();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[ClientBehavior %s] Exception on Entry - continuing with next client behavior. Exception info: %s",
                  cbname, e.what());
      }
    }
  }
  else
  {
    ROS_INFO("Orthogonal %s OnEntry", this->getName().c_str());
  }
}

void ISmaccOrthogonal::onExit()
{
  if (clientBehaviors_.size() > 0)
  {
    for (auto &clBehavior : clientBehaviors_)
    {
      auto cbname = clBehavior->getName().c_str();
      ROS_INFO("Orthogonal %s OnExit, current Behavior: %s", this->getName().c_str(), cbname);
      try
      {
        clBehavior->onExit();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[ClientBehavior %s] Exception onExit - continuing with next client behavior. Exception info: %s",
                  cbname, e.what());
      }
    }
    clientBehaviors_.clear();
  }
  else
  {
    ROS_INFO("Orthogonal %s OnExit", this->getName().c_str());
  }
}
}  // namespace smacc