#include <smacc/smacc_state_machine_info.h>
#include <smacc/reflection.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
void SmaccStateMachineInfo::assembleSMStructureMessage(ISmaccStateMachine *sm)
{
    ROS_INFO("----------- PRINT STATE MACHINE STRUCTURE -------------------");
    stateMsgs.clear();
    for (auto &val : this->states)
    {
        smacc_msgs::SmaccState stateMsg;
        auto state = val.second;
        stateMsg.index = state->stateIndex_;

        std::stringstream ss;
        ss << "**** State: " << demangleSymbol(val.first.c_str()) << std::endl;

        stateMsg.name = state->getDemangledFullName();
        stateMsg.level = (int)state->getStateLevel();

        ss << "**** State: " << stateMsg.name << std::endl;

        ss << "Index: " << stateMsg.index << std::endl;
        ss << "StateLevel: " << stateMsg.level << std::endl;

        ss << " Childstates:" << std::endl;

        for (auto &child : state->children_)
        {
            auto childStateName = child->getDemangledFullName();
            stateMsg.children_states.push_back(childStateName);

            ss << " - " << childStateName << std::endl;
        }

        ss << " Transitions:" << std::endl;

        for (auto &transition : state->transitions_)
        {
            smacc_msgs::SmaccTransition transitionMsg;

            transitionInfoToMsg(transition, transitionMsg);

            ss << " - Transition.  " << std::endl;
            ss << "      - Index: " << transitionMsg.index << std::endl;
            ss << "      - Transition Name: " << transitionMsg.transition_name << std::endl;
            ss << "      - Transition Type: " << transitionMsg.transition_type << std::endl;
            ss << "      - Event Type :" << transitionMsg.event.event_type << std::endl;
            ss << "      - Event Source: " << transitionMsg.event.event_source << std::endl;
            ss << "      - Event ObjectTag: " << transitionMsg.event.event_object_tag << std::endl;
            ss << "      - Event Label: " << transitionMsg.event.label << std::endl;
            ss << "      - Destiny State: " << transitionMsg.destiny_state_name << std::endl;
            ss << "      - Owner State: " << transitionMsg.destiny_state_name << std::endl;
            ss << "      - Is History Node: " << std::to_string(transitionMsg.history_node) << std::endl;
            ss << "      - TransitionC++Type: " << transition.transitionTypeInfo->getFullName() << std::endl;
            ss << "      - EventC++Type: " << transition.eventInfo->eventType->getFullName() << std::endl;

            stateMsg.transitions.push_back(transitionMsg);
        }

        const std::type_info *statetid = state->tid_;

        std::map<const std::type_info *, std::vector<smacc::StateBehaviorInfoEntry *>> smaccBehaviorInfoMappingByOrthogonalType;

        ss << " Orthogonals:" << std::endl;
        if (SmaccStateInfo::staticBehaviorInfo.count(statetid) > 0)
        {
            for (auto &bhinfo : SmaccStateInfo::staticBehaviorInfo[statetid])
            {
                if (smaccBehaviorInfoMappingByOrthogonalType.count(bhinfo.orthogonalType) == 0)
                {
                    smaccBehaviorInfoMappingByOrthogonalType[bhinfo.orthogonalType] = std::vector<smacc::StateBehaviorInfoEntry *>();
                }

                smaccBehaviorInfoMappingByOrthogonalType[bhinfo.orthogonalType].push_back(&bhinfo);
            }
        }

        auto &runtimeOrthogonals = sm->getOrthogonals();

        for (auto &orthogonal : runtimeOrthogonals)
        {
            smacc_msgs::SmaccOrthogonal orthogonalMsg;

            const auto *orthogonaltid = &typeid(*(orthogonal.second));
            orthogonalMsg.name = demangleSymbol(orthogonaltid->name());

            ss << " - orthogonal: " << orthogonalMsg.name << std::endl;

            if (smaccBehaviorInfoMappingByOrthogonalType[orthogonaltid].size() > 0)
            {
                auto &behaviors = smaccBehaviorInfoMappingByOrthogonalType[orthogonaltid];
                for (auto &bhinfo : behaviors)
                {
                    auto substateBehaviorName = demangleSymbol(bhinfo->behaviorType->name());
                    orthogonalMsg.substate_behavior_names.push_back(substateBehaviorName);
                    ss << "          - substate behavior: " << substateBehaviorName << std::endl;
                }
            }
            else
            {
                ss << "          - NO SUBSTATE BEHAVIORS -" << std::endl;
            }

            auto &clients = orthogonal.second->getClients();
            if (clients.size() > 0)
            {
                for (auto *client : clients)
                {
                    auto clientTid = &(typeid(*client));
                    auto clientName = demangleSymbol(clientTid->name());
                    orthogonalMsg.client_names.push_back(clientName);
                    ss << "          - client: " << clientName << std::endl;
                }
            }
            else
            {
                ss << "          - NO CLIENTS - " << std::endl;
            }
            stateMsg.orthogonals.push_back(orthogonalMsg);
        }

        ss << " Logic units:" << std::endl;
        if (SmaccStateInfo::logicUnitsInfo.count(statetid) > 0)
        {
            int k = 0;
            for (auto &luinfo : SmaccStateInfo::logicUnitsInfo[statetid])
            {
                smacc_msgs::SmaccLogicUnit logicUnitMsg;
                logicUnitMsg.index = k++;
                logicUnitMsg.type_name = demangleSymbol(luinfo.logicUnitType->name());

                ss << " - logic unit: " << logicUnitMsg.type_name << std::endl;
                if (luinfo.objectTagType != nullptr)
                {
                    logicUnitMsg.object_tag = luinfo.objectTagType->getFullName();
                    ss << "        - object tag: " << logicUnitMsg.object_tag << std::endl;
                }

                for (auto &tev : luinfo.sourceEventTypes)
                {
                    // WE SHOULD CREATE A SMACC_EVENT_INFO TYPE, also using in typewalker transition
                    auto eventTypeName = tev->getEventTypeName();
                    smacc_msgs::SmaccEvent event;

                    ss << "             - triggering event: " << tev->getEventTypeName() << std::endl;
                    event.event_type = eventTypeName;

                    event.event_source = tev->getEventSourceName();
                    ss << "                 - source type: " << event.event_source << std::endl;

                    event.event_object_tag = tev->getObjectTagName();
                    ss << "                 - source object: " << event.event_object_tag << std::endl;

                    event.label = tev->label;
                    ss << "                 - event label: " << event.label << std::endl;

                    logicUnitMsg.event_sources.push_back(event);
                }

                stateMsg.logic_units.push_back(logicUnitMsg);
            }
        }
        else
        {
            ss << "- NO LOGIC UNITS - " << std::endl;
        }

        ROS_INFO_STREAM(ss.str());
        stateMsgs.push_back(stateMsg);
    }

    std::sort(stateMsgs.begin(), stateMsgs.end(), [](auto &a, auto &b) {
        return a.index > b.index;
    });
}
} // namespace smacc