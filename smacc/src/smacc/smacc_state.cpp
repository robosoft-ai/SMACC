#include <smacc/smacc_state.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
std::string ISmaccState::getClassName()
{
    return demangleSymbol(typeid(*this).name());
}

void ISmaccState::notifyTransitionFromTransitionTypeInfo(smacc::TypeInfo::Ptr &transitionType)
{
    ROS_ERROR_STREAM("NOTIFY TRANSITION: " << transitionType->getFullName());

    auto currstateinfo = this->getStateMachine().getCurrentStateInfo();
    if (currstateinfo != nullptr)
    {
        //ROS_ERROR_STREAM("CURRENT STATE INFO: " << currstateinfo->fullStateName);
        for (auto &transition : currstateinfo->transitions_)
        {
            std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
            //ROS_ERROR_STREAM("candidate transition: " << transitionCandidateName);

            if (transitionType->getFullName() == transitionCandidateName)
            {
                this->getStateMachine().publishTransition(transition);
                return;
            }
        }
        // debug information if not found
        ROS_ERROR_STREAM("Transition happened, but there is not any transitioninfo match available");
        for (auto &transition : currstateinfo->transitions_)
        {
            std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
            ROS_ERROR_STREAM("candidate transition: " << transitionCandidateName);
        }
    }
    else
    {
        ROS_ERROR_STREAM("Transition happened, but current state was not set. Transition candidates:");
    }
}

} // namespace smacc