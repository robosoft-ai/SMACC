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
      ROS_INFO("[Orthogonal %s] adding client behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());
      clBehavior->stateMachine_ = this->getStateMachine();
      clBehavior->currentOrthogonal = this;

      clientBehaviors_.push_back(clBehavior);
    }
    else
    {
      ROS_INFO("[orthogonal %s] no client behaviors in this state", this->getName().c_str());
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
      ROS_INFO("[Orthogonal %s] runtimeConfigure, current Behavior: %s", this->getName().c_str(),
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
        ROS_INFO("[Orthogonal %s] OnEntry, current Behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());

        try
        {
          clBehavior->onEntry();
        }
        catch (const std::exception &e)
        {
          ROS_ERROR("[ClientBehavior %s] Exception on Entry - continuing with next client behavior. Exception info: %s",
                    clBehavior->getName().c_str(), e.what());
        }
      }
    }
    else
    {
      ROS_INFO("[Orthogonal %s] OnEntry", this->getName().c_str());
    }
  }

  void ISmaccOrthogonal::onExit()
  {
    if (clientBehaviors_.size() > 0)
    {
      for (auto &clBehavior : clientBehaviors_)
      {
        ROS_INFO("[Orthogonal %s] OnExit, current Behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());
        try
        {
          clBehavior->onExit();
        }
        catch (const std::exception &e)
        {
          ROS_ERROR("[ClientBehavior %s] Exception onExit - continuing with next client behavior. Exception info: %s", clBehavior->getName().c_str(), e.what());
        }
      }
      clientBehaviors_.clear();
    }
    else
    {
      ROS_INFO("[Orthogonal %s] OnExit", this->getName().c_str());
    }
  }
} // namespace smacc