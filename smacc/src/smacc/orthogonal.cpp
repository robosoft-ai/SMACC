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

    void ISmaccOrthogonal::addClientBehavior(std::shared_ptr<smacc::ISmaccClientBehavior> clBehavior)
    {
        if (clBehavior != nullptr)
        {
            ROS_INFO("[Orthogonal %s] adding client behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());
            clBehavior->stateMachine_ = this->getStateMachine();
            clBehavior->currentOrthogonal = this;

            clientBehaviors_.back().push_back(clBehavior);
        }
        else
        {
            ROS_INFO("[orthogonal %s] no client behaviors in this state", this->getName().c_str());
        }
    }

    void ISmaccOrthogonal::onInitialize()
    {
    }

    void ISmaccOrthogonal::initState(ISmaccState *state)
    {
        ROS_INFO("[Orthogonal %s] initState: %s", this->getName().c_str(), state->getClassName().c_str());
        clientBehaviors_.push_back(std::vector<std::shared_ptr<smacc::ISmaccClientBehavior>>());
    }

    std::string ISmaccOrthogonal::getName() const
    {
        return demangleSymbol(typeid(*this).name());
    }

    void ISmaccOrthogonal::runtimeConfigure()
    {
        for (auto &clBehavior : clientBehaviors_.back())
        {
            ROS_INFO("[Orthogonal %s] runtimeConfigure, current Behavior: %s", this->getName().c_str(),
                     clBehavior->getName().c_str());

            clBehavior->runtimeConfigure();
        }
    }

    void ISmaccOrthogonal::onEntry()
    {
        if (clientBehaviors_.back().size() > 0)
        {
            for (auto &clBehavior : clientBehaviors_.back())
            {
                ROS_INFO("[Orthogonal %s] OnEntry, current Behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());

                try
                {
                    clBehavior->executeOnEntry();
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
        if (clientBehaviors_.back().size() > 0)
        {
            for (auto &clBehavior : clientBehaviors_.back())
            {
                ROS_INFO("[Orthogonal %s] OnExit, current Behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());
                try
                {
                    clBehavior->executeOnExit();
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("[ClientBehavior %s] Exception onExit - continuing with next client behavior. Exception info: %s", clBehavior->getName().c_str(), e.what());
                }
            }
        }
        else
        {
            ROS_INFO("[Orthogonal %s] OnExit", this->getName().c_str());
        }
    }

    void ISmaccOrthogonal::onDispose()
    {
        for (auto &clBehavior : clientBehaviors_.back())
        {
            clBehavior->dispose();
            this->getStateMachine()->disconnectSmaccSignalObject((void*)clBehavior.get());
        }

        clientBehaviors_.back().clear();
        clientBehaviors_.pop_back();
    }
} // namespace smacc
