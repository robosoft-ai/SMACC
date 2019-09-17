#pragma once

#include <memory>
#include <map>
#include <smacc/common.h>

namespace smacc
{

class SmaccStateMachineInfo : public std::enable_shared_from_this<SmaccStateMachineInfo>
{
public:
    std::map<std::string, std::shared_ptr<SmaccStateInfo>> states;

    template <typename InitialStateType>
    void buildStateMachineInfo();

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> createState(std::shared_ptr<SmaccStateInfo> parentState);

    template <typename StateType>
    bool containsState()
    {
        auto typeNameStr = typeid(StateType).name();

        return states.count(typeNameStr) > 0;
    }

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> getState()
    {
        if (this->containsState<StateType>())
        {
            return states[typeid(StateType).name()];
        }
        return nullptr;
    }

    template <typename StateType>
    void addState(std::shared_ptr<StateType> &state);

    void printAllStates()
    {
        for (auto &val : this->states)
        {
            auto state = val.second;

            std::stringstream ss;
            if (state->parentState_ == nullptr)
            {
                ss << "**** Print StateType: " << state->demangledStateName << std::endl;
                ss << " Childstates:" << std::endl;

                for (auto &child : state->children_)
                {
                    ss << " - " << child->demangledStateName << std::endl;
                }

                ss << " Transitions:" << std::endl;

                for (auto &transition : state->transitions_)
                {
                    ss << " - " << transition.first << " -> " << transition.second->demangledStateName << std::endl;
                }

                ROS_INFO_STREAM(ss.str());
            }
        }
    }
};

//--------------------------------

template <typename T>
struct type_
{
    using type = T;
};

template <typename T>
struct add_type_wrapper
{
    using type = type_<T>;
};
//---------------------------------------------

struct AddSubState
{
    std::shared_ptr<SmaccStateInfo> &parentState_;
    AddSubState(std::shared_ptr<SmaccStateInfo> &parentState)
        : parentState_(parentState)
    {
    }

    template <typename T>
    void operator()(T);
};

struct AddTransition
{
    std::shared_ptr<SmaccStateInfo> &currentState_;

    AddTransition(std::shared_ptr<SmaccStateInfo> &currentState)
        : currentState_(currentState)
    {
    }

    template <typename T>
    void operator()(T);
};

template <typename InitialStateType>
struct WalkStatesExecutor
{
    static void walkStates(std::shared_ptr<SmaccStateInfo> &currentState, bool rootInitialNode);
};

template <typename T>
void AddSubState::operator()(T)
{
    using type_t = typename T::type;
    //auto childState = this->parentState_->createChildState<type_t>()
    WalkStatesExecutor<type_t>::walkStates(parentState_, false);
}
//--------------------------------------------
template <typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type
processSubState(std::shared_ptr<SmaccStateInfo> &parentState)
{
    WalkStatesExecutor<T>::walkStates(parentState, false);
}

template <typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type
processSubState(std::shared_ptr<SmaccStateInfo> &parentState)
{
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<T, add_type_wrapper<_1>>::type;
    boost::mpl::for_each<wrappedList>(AddSubState(parentState));
}
//--------------------------------------------
template <typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type
processTransitions(std::shared_ptr<SmaccStateInfo> &sourceState)
{
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<T, add_type_wrapper<_1>>::type;
    boost::mpl::for_each<wrappedList>(AddTransition(sourceState));
}

template <typename Ev, typename Dst>
void processTransition(statechart::transition<Ev, Dst> *, std::shared_ptr<SmaccStateInfo> &sourceState)
{
    //ROS_INFO_STREAM("GOTCHA");

    if (!sourceState->stateMachine_->containsState<Dst>())
    {
        auto siblingnode = sourceState->stateMachine_->createState<Dst>(sourceState->parentState_);
        WalkStatesExecutor<Dst>::walkStates(siblingnode, true);
        sourceState->declareTransition<Ev>(siblingnode);
    }
    else
    {
        auto siblingnode = sourceState->stateMachine_->getState<Dst>();
        sourceState->declareTransition<Ev>(siblingnode);
    }
}

template <typename EvType>
void SmaccStateInfo::declareTransition(std::shared_ptr<SmaccStateInfo> &dstState)
{
    auto evtype = demangledTypeName<EvType>();
    transitions_[evtype] = dstState;
}

template <typename Ev>
void processTransition(statechart::custom_reaction<Ev> *, std::shared_ptr<SmaccStateInfo> &sourceState)
{
    //ROS_INFO_STREAM("GOTCHA");
}

template <typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type
processTransitions(std::shared_ptr<SmaccStateInfo> &sourceState)
{
    //ROS_INFO_STREAM("state transition from: " << sourceState->demangledStateName << " of type: " << demangledTypeName<T>());
    T *dummy;
    processTransition(dummy, sourceState);
}
//--------------------------------------------
template <typename T>
void AddTransition::operator()(T)
{
    using type_t = typename T::type;
    processTransitions<type_t>(currentState_);
}

template <typename InitialStateType>
void WalkStatesExecutor<InitialStateType>::walkStates(std::shared_ptr<SmaccStateInfo> &parentState, bool rootInitialNode)
{
    //ros::Duration(1).sleep();
    auto currentname = demangledTypeName<InitialStateType>();

    std::shared_ptr<SmaccStateInfo> targetState;

    if (!rootInitialNode)
    {
        if (parentState->stateMachine_->containsState<InitialStateType>())
        {
            // it already exist: break;
            return;
        }

        targetState = parentState->createChildState<InitialStateType>();
    }
    else
    {
        targetState = parentState;
    }

    typedef typename std::remove_pointer<decltype(InitialStateType::smacc_inner_type)>::type InnerType;
    processSubState<InnerType>(targetState);

    // -------------------- REACTIONS --------------------
    typedef typename InitialStateType::reactions reactions;
    //ROS_INFO_STREAM("state machine initial state reactions: "<< demangledTypeName<reactions>());

    processTransitions<reactions>(targetState);
}

template <typename InitialStateType>
void SmaccStateMachineInfo::buildStateMachineInfo()
{
    auto initialState = this->createState<InitialStateType>(nullptr);
    WalkStatesExecutor<InitialStateType>::walkStates(initialState, true);
}

template <typename StateType>
std::shared_ptr<SmaccStateInfo> SmaccStateMachineInfo::createState(std::shared_ptr<SmaccStateInfo> parent)
{
    auto thisptr = this->shared_from_this();
    auto state = std::shared_ptr<SmaccStateInfo>(new SmaccStateInfo(parent, thisptr));
    state->demangledStateName = demangledTypeName<StateType>();
    state->fullStateName = typeid(StateType).name();

    std::vector<std::string> strs;
    boost::split(strs, state->demangledStateName, boost::is_any_of(":"));

    state->demangledStateName = strs.back();

    if (parent != nullptr)
    {
        parent->children_.push_back(state);
    }

    this->addState(state);

    return state;
}

template <typename StateType>
void SmaccStateMachineInfo::addState(std::shared_ptr<StateType> &state)
{
    states[state->fullStateName] = state;
}

template <typename StateType>
std::shared_ptr<SmaccStateInfo> SmaccStateInfo::createChildState()
{
    auto childState = this->stateMachine_->createState<StateType>(shared_from_this());
    //this->stateMachine_->addState(childState);
    //stateMachineInfo.addState(stateMachineInfo)
    //stateNames.push_back(currentname);
    //ROS_INFO("------------");
    //ROS_INFO_STREAM("** STATE state: "<< this->demangledStateName);

    return childState;
}
}