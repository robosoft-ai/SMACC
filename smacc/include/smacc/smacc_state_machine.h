/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/common.h>

#include <boost/any.hpp>
#include <map>
#include <mutex>

#include <type_traits>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/list.hpp>
#include <boost/utility/enable_if.hpp>


namespace smacc
{

class SmaccStateMachineInfo;
class SmaccStateInfo;
class Orthogonal;
class ISmaccState;


// This class describes the concept of Smacc State Machine in an abastract way.
// The SmaccStateMachineBase inherits from this state machine and from 
// statechart::StateMachine<> (via multiple inheritance)
class ISmaccStateMachine
{
    public:
    
    std::shared_ptr<SmaccStateMachineInfo> info_;
    ISmaccStateMachine( SignalDetector* signalDetector);

    virtual ~ISmaccStateMachine();

    void notifyOnStateEntry(ISmaccState* state);

    void notifyOnStateExit(ISmaccState* state);

    template <typename TOrthogonal>
    void getOrthogonal(TOrthogonal*& storage);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh, std::string value);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh=ros::NodeHandle())
    {
       requiresComponent(storage, nh, "");
    }

    template <typename EventType>
    void postEvent( EventType* ev);

    template <typename T>
    bool getGlobalSMData(std::string name, T& ret)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        //ROS_WARN("get SM Data lock acquire");
        bool success = false;

        if(!globalData_.count(name))
        {
            //ROS_WARN("get SM Data - data do not exist");
            success = false;
        }
        else
        {
            //ROS_WARN("get SM DAta -data exist. accessing");
            try
            {
                boost::any v = globalData_[name];
                //ROS_WARN("get SM DAta -data exist. any cast");
                ret = boost::any_cast<T>(v);
                success = true;    
                //ROS_WARN("get SM DAta -data exist. success");
            }
            catch(boost::bad_any_cast& ex)
            {
                ROS_ERROR("bad any cast: %s", ex.what());
                success = false;
            }
        }

        //ROS_WARN("get SM Data lock release");
        return success;
    }

    template <typename T>
    void setGlobalSMData(std::string name, T value)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        //ROS_WARN("set SM Data lock acquire");
        globalData_[name] = value;
    }

    /// Used by the ISMaccActionClients when a new send goal is launched
    void registerActionClientRequest(ISmaccActionClient* component);

    template <typename StateField, typename BehaviorType> 
    void mapBehavior()
    {
        std::string stateFieldName =demangleSymbol(typeid(StateField).name());
        std::string behaviorType =demangleSymbol(typeid(BehaviorType).name());
        ROS_INFO("Mapping state field '%s' to stateBehavior '%s'", stateFieldName.c_str(), behaviorType.c_str());
        SmaccStateBehavior* globalreference;
        if(!this->getGlobalSMData(stateFieldName,globalreference))
        {
            // Using the requires component approach, we force a unique existence
            // of this component
            BehaviorType* behavior;
            this->requiresComponent(behavior);
            globalreference =  dynamic_cast<SmaccStateBehavior*>(behavior);

            this->setGlobalSMData(stateFieldName, globalreference);
        }
    }

private:

    std::mutex m_mutex_;
    
    std::map<std::string, smacc::ISmaccComponent*> plugins_;

    std::map<std::string, boost::any> globalData_;

    std::map<std::string, smacc::Orthogonal*> orthogonals_;

    // Event to notify to the signaldetection thread that a request has been created...
    SignalDetector* signalDetector_;
};


//--------------------------------
class SmaccStateInfo: public std::enable_shared_from_this<SmaccStateInfo>
{
public:
    bool active_;
    std::string fullStateName;
    std::string demangledStateName;

    std::shared_ptr<SmaccStateMachineInfo> stateMachine_;
    std::shared_ptr<SmaccStateInfo> parentState_;
    std::map<std::string,std::shared_ptr<SmaccStateInfo>> transitions_;

    std::vector<std::shared_ptr<SmaccStateInfo>> children_;
    int depth_;

    SmaccStateInfo(std::shared_ptr<SmaccStateInfo> parentState, std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo)
    {
        parentState_ = parentState;
        stateMachine_ = stateMachineInfo;

        if(parentState_!=nullptr)
            depth_ = parentState->depth_ +1;
    }

    inline int depth() const{return depth_;}


    std::string getFullPath()
    {
        if( parentState_==nullptr)
            return this->toShortName();
        else
            return this->parentState_->getFullPath() + "/" + this->toShortName();
    }

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> createChildState();

    template <typename EvType>
    void declareTransition(std::shared_ptr<SmaccStateInfo>& dstState);

    const std::string& toShortName() const
    {
        return this->demangledStateName;
    }
};


class SmaccStateMachineInfo: public std::enable_shared_from_this<SmaccStateMachineInfo>
{
public:
    std::map<std::string,std::shared_ptr<SmaccStateInfo>> states;

    template <typename InitialStateType>
    void buildStateMachineInfo();

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> createState(std::shared_ptr<SmaccStateInfo> parentState);

    template <typename StateType>
    bool containsState()
    {
        auto typeNameStr = typeid(StateType).name();

        return states.count(typeNameStr)>0;
    }

    template <typename StateType>
    std::shared_ptr<SmaccStateInfo> getState()
    {
        if( this->containsState<StateType>())
        {
            return states[typeid(StateType).name()];
        }
        return nullptr;
    }

    template <typename StateType>
    void addState(std::shared_ptr<StateType>& state);


    void printAllStates()
    {
        for(auto& val: this->states)
        {
            auto state = val.second;

            std::stringstream ss;
            if(state->parentState_== nullptr)
            {
                ss << "**** Print StateType: " << state->demangledStateName <<std::endl;
                ss << " Childstates:" << std::endl;

                for(auto& child: state->children_)
                {
                    ss << " - " << child->demangledStateName << std::endl;
                }

                ss << " Transitions:" << std::endl;

                for(auto& transition: state->transitions_)
                {
                    ss << " - " << transition.first << " -> " << transition.second->demangledStateName << std::endl;
                }

                ROS_INFO_STREAM(ss.str());
            }
        }
    }
};

//--------------------------------


template<typename T>
struct type_
{
    using type= T;
};

template <typename T>
struct add_type_wrapper
{
    using type = type_<T>;
};
//---------------------------------------------

struct AddSubState
{
    std::shared_ptr<SmaccStateInfo>& parentState_;
    AddSubState(std::shared_ptr<SmaccStateInfo>& parentState)
    : parentState_(parentState)
    {
    }

    template <typename T> 
    void operator()(T);
};

struct AddTransition
{
    std::shared_ptr<SmaccStateInfo>& currentState_;
    
    AddTransition(std::shared_ptr<SmaccStateInfo>& currentState)
    : currentState_(currentState)
    {
    }

    template <typename T>
    void operator()(T);
};

template <typename InitialStateType>
struct WalkStatesExecutor
{
    static void walkStates(std::shared_ptr<SmaccStateInfo>& currentState, bool rootInitialNode);
};

template <typename T> 
void AddSubState::operator()(T)
{
    using type_t = typename T::type;
    //auto childState = this->parentState_->createChildState<type_t>()
    WalkStatesExecutor<type_t>::walkStates(parentState_, false);   
}
//--------------------------------------------
template<typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type
processSubState(std::shared_ptr<SmaccStateInfo>& parentState)
{
    WalkStatesExecutor<T>::walkStates(parentState, false);
}

template<typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type
processSubState(std::shared_ptr<SmaccStateInfo>& parentState)
{
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<T,add_type_wrapper<_1>>::type;
    boost::mpl::for_each<wrappedList>(AddSubState(parentState));
}
//--------------------------------------------
template<typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type
processTransitions(std::shared_ptr<SmaccStateInfo>& sourceState)
{
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<T,add_type_wrapper<_1>>::type;
    boost::mpl::for_each<wrappedList>(AddTransition(sourceState));
}

template<typename Ev, typename Dst>
void processTransition(statechart::transition<Ev,Dst>* , std::shared_ptr<SmaccStateInfo>& sourceState)
{
    //ROS_INFO_STREAM("GOTCHA");

    if(!sourceState->stateMachine_->containsState<Dst>())
    {
        auto siblingnode = sourceState->stateMachine_->createState<Dst>(sourceState->parentState_);
        WalkStatesExecutor<Dst>::walkStates(siblingnode, true);
        sourceState->declareTransition< Ev>(siblingnode);
    }
    else
    {
        auto siblingnode = sourceState->stateMachine_->getState<Dst>();
        sourceState->declareTransition< Ev>(siblingnode);
    }
}

template <typename EvType>
void SmaccStateInfo::declareTransition(std::shared_ptr<SmaccStateInfo>& dstState)
{
    auto evtype = demangledTypeName<EvType>();
    transitions_[evtype] = dstState;
}

template<typename Ev>
void processTransition(statechart::custom_reaction<Ev>* , std::shared_ptr<SmaccStateInfo>& sourceState)
{
    //ROS_INFO_STREAM("GOTCHA");
}

template<typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type
processTransitions(std::shared_ptr<SmaccStateInfo>& sourceState)
{
    //ROS_INFO_STREAM("state transition from: " << sourceState->demangledStateName << " of type: " << demangledTypeName<T>());
    T* dummy;
    processTransition(dummy,sourceState);


}
//--------------------------------------------
template <typename T> 
void AddTransition::operator()(T)
{
    using type_t = typename T::type;
    processTransitions<type_t>(currentState_);
}

template <typename InitialStateType>
  void WalkStatesExecutor<InitialStateType>::walkStates(std::shared_ptr<SmaccStateInfo>& parentState, bool rootInitialNode)
{
    //ros::Duration(1).sleep();
    auto currentname = demangledTypeName<InitialStateType>();

    std::shared_ptr<SmaccStateInfo> targetState;

    if(!rootInitialNode)
    {
        if(parentState->stateMachine_->containsState<InitialStateType>())
        {
            // it already exist: break;
            return;
        }
    
        targetState  = parentState->createChildState<InitialStateType>();
    }
    else
    {
        targetState  = parentState;
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
    boost::split(strs,state->demangledStateName,boost::is_any_of(":"));

    state->demangledStateName = strs.back();

    if(parent!=nullptr)
    {
        parent->children_.push_back(state);
    }

    this->addState(state);

    return state;
}

template <typename StateType>
void SmaccStateMachineInfo::addState(std::shared_ptr<StateType>& state)
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
