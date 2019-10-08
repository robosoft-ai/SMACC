#include <smacc/smacc_state_info.h>

namespace smacc
{

std::map<const std::type_info *, std::shared_ptr<std::vector<StateBehaviorInfoEntry>>> SmaccStateInfo::staticBehaviorInfo;

SmaccStateInfo::SmaccStateInfo(std::shared_ptr<SmaccStateInfo> parentState, std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo)
{
    parentState_ = parentState;
    stateMachine_ = stateMachineInfo;

    if (parentState_ != nullptr)
        depth_ = parentState->depth_ + 1;
}

void SmaccStateInfo::getAncestors(std::list<std::shared_ptr<SmaccStateInfo>> &ancestorsList)
{
    ancestorsList.push_front(shared_from_this());
    if (parentState_ != nullptr)
    {
        this->parentState_->getAncestors(ancestorsList);
    }
}

std::string SmaccStateInfo::getFullPath()
{
    if (parentState_ == nullptr)
        return this->toShortName();
    else
        return this->parentState_->getFullPath() + "/" + this->toShortName();
}

const std::string &SmaccStateInfo::toShortName() const
{
    return this->demangledStateName;
}

}