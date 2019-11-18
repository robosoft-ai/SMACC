#include <smacc/reflection.h>
#include <ros/ros.h>
#include <smacc/common.h>
#include <smacc/string_type_walker.h>

namespace smacc
{

std::map<const std::type_info *, std::vector<StateBehaviorInfoEntry>> SmaccStateInfo::staticBehaviorInfo;
std::map<const std::type_info *, std::vector<SmaccLogicUnitInfo>> SmaccStateInfo::logicUnitsInfo;

SmaccStateInfo::SmaccStateInfo(const std::type_info *tid, std::shared_ptr<SmaccStateInfo> parentState, std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo)
{
    tid_ = tid;
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

SmaccStateType SmaccStateInfo::getStateLevel()
{
    if (this->children_.size() == 0)
    {
        if (this->parentState_ != nullptr)
        {
            return SmaccStateType::SUPERSTATE_ROUTINE;
        }
        else
        {
            return SmaccStateType::STATE;
        }
    }
    else
    {
        return SmaccStateType::SUPERSTATE;
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

std::string SmaccStateInfo::getDemangledFullName() const
{
    return demangleSymbol(this->fullStateName.c_str());
}
//---------------------------------------------
SmaccEventInfo::SmaccEventInfo(std::shared_ptr<smacc::TypeInfo> eventType)
{
    ROS_INFO_STREAM("CREATING EVENT INFO: " << eventType->getFullName());

    this->eventType = eventType;
}

std::string SmaccEventInfo::getEventSourceName()
{
    if (eventType->templateParameters.size() > 0)
    {
        auto eventsourcename = demangleSymbol(eventType->templateParameters[0]->getFullName().c_str());
        ROS_ERROR_STREAM("EVENT SOURCE NAME -------------" << eventsourcename);
        return eventsourcename;
    }
    else
    {
        return "";
    }
}

std::string SmaccEventInfo::getEventTypeName()
{
    return demangleSymbol(eventType->getNonTemplatetypename().c_str());
}

std::string SmaccEventInfo::getObjectTagName()
{
    if (eventType->templateParameters.size() > 1)
    {
        return demangleSymbol(eventType->templateParameters[1]->getFullName().c_str());
    }
    else
    {
        return "";
    }
}

} // namespace smacc