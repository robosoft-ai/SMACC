#include <smacc/orthogonal.h>
#include <smacc/impl/smacc_state_machine_impl.h>
#include <smacc/smacc_substate_behavior.h>

namespace smacc
{
void Orthogonal::setStateMachine(ISmaccStateMachine *value)
{
    this->stateMachine_ = value;
    this->onInitialize();
}

void Orthogonal::setStateBehavior(std::shared_ptr<smacc::SmaccSubStateBehavior> statebehavior)
{
    if (statebehavior != nullptr)
    {
        ROS_INFO("Setting Ortho %s State behavior: %s", this->getName().c_str(), statebehavior->getName().c_str());
        statebehavior->stateMachine = this->stateMachine_;
        statebehavior->currentOrthogonal = this;
        
        currentBehavior_ = statebehavior;
    }
    else
    {
        ROS_INFO("Not behavioral State by orthogonal");
    }
}

void Orthogonal::onInitialize()
{

}

std::string Orthogonal::getName() const
{
    return demangleSymbol(typeid(*this).name());
}

void Orthogonal::onEntry()
{
    if (currentBehavior_ != nullptr)
    {
        ROS_INFO("Orthogonal %s OnEntry, current Behavior: %s",
                 this->getName().c_str(),
                 currentBehavior_->getName().c_str());

        currentBehavior_->onEntry();
    }
    else
    {
        ROS_INFO("Orthogonal %s OnEntry",
                 this->getName().c_str());
    }
}

void Orthogonal::onExit()
{
    if (currentBehavior_ != nullptr)
    {
        ROS_INFO("Orthogonal %s OnExit, current Behavior: %s", this->getName().c_str(), currentBehavior_->getName().c_str());
        currentBehavior_->onExit();
        currentBehavior_ = nullptr;
    }
    else
    {
        ROS_INFO("Orthogonal %s OnExit", this->getName().c_str());
    }
}
} // namespace smacc