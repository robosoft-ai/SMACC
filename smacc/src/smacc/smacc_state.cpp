#include <smacc/smacc_state.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
std::string ISmaccState::getClassName()
{
    return demangleSymbol(typeid(*this).name());
}

void ISmaccState::notifyTransitionFromTransitionTypeInfo(TypeInfo::Ptr &transitionType)
{
    ROS_INFO_STREAM("NOTIFY TRANSITION: " << transitionType->getFullName());

    //auto currstateinfo = this->getStateMachine().getCurrentStateInfo();
    auto currstateinfo = this->stateInfo_;

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
        ROS_ERROR_STREAM("Transition happened, from current state " << currstateinfo->getDemangledFullName() << " but there is not any transitioninfo match available to publish transition: " << transitionType->getFullName());
        std::stringstream ss;

        auto stateinfo = currstateinfo;

        for (auto &transition : currstateinfo->transitions_)
        {
            std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
            ROS_ERROR_STREAM("- candidate transition: " << transitionCandidateName);
        }

        ROS_ERROR("Ancestors candidates: ");

        std::list<const SmaccStateInfo *> ancestors;
        stateinfo->getAncestors(ancestors);

        for (auto &ancestor : ancestors)
        {
            ROS_ERROR_STREAM(" * Ancestor " << ancestor->getDemangledFullName() << ":");
            for (auto &transition : ancestor->transitions_)
            {
                std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
                ROS_ERROR_STREAM("- candidate transition: " << transitionCandidateName);
                if (transitionType->getFullName() == transitionCandidateName)
                {
                    ROS_ERROR("GOTCHA");
                }
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("Transition happened, but current state was not set. Transition candidates:");
    }
}

} // namespace smacc