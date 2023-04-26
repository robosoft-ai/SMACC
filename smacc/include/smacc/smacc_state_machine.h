/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <boost/any.hpp>
#include <map>
#include <mutex>
#include <stack>

#include <smacc/common.h>
#include <smacc/introspection/introspection.h>
#include <smacc/introspection/smacc_state_machine_info.h>
#include <smacc/smacc_updatable.h>
#include <smacc/smacc_signal.h>

#include <smacc_msgs/SmaccStateMachine.h>
#include <smacc_msgs/SmaccTransitionLogEntry.h>
#include <smacc_msgs/SmaccStatus.h>
#include <smacc_msgs/SmaccGetTransitionHistory.h>

#include <smacc/smacc_state.h>
#include <smacc/smacc_state_reactor.h>
//#include <smacc/smacc_event_generator.h>
#include <smacc/callback_counter_semaphore.h>

namespace smacc
{

using namespace smacc::introspection;

enum class EventLifeTime{
    ABSOLUTE,
    CURRENT_STATE /*events are discarded if we are leaving the state it were created. I is used for client behaviors whose lifetime is associated to state*/
};

enum class StateMachineInternalAction
{
    STATE_CONFIGURING,
    STATE_ENTERING,
    STATE_STEADY,
    STATE_EXITING,
    TRANSITIONING
};



// This class describes the concept of Smacc State Machine in an abastract way.
// The SmaccStateMachineBase inherits from this state machine and from
// statechart::StateMachine<> (via multiple inheritance)
class ISmaccStateMachine
{
public:
    ISmaccStateMachine(SignalDetector *signalDetector);

    virtual ~ISmaccStateMachine();

    virtual void reset();

    virtual void stop();

    virtual void eStop();

    template <typename TOrthogonal>
    TOrthogonal *getOrthogonal();

    const std::map<std::string, std::shared_ptr<smacc::ISmaccOrthogonal>> &getOrthogonals() const;

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage);

    template <typename SmaccComponentType>
    void requiresComponent(std::shared_ptr<SmaccComponentType> &storage);

    template <typename EventType>
    void postEvent(EventType *ev, EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);

    template <typename EventType>
    void postEvent(EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);

    void getTransitionLogHistory();

    template <typename T>
    bool getGlobalSMData(std::string name, T &ret);

    template <typename T>
    void setGlobalSMData(std::string name, T value);

    template <typename StateField, typename BehaviorType>
    void mapBehavior();

    std::string getStateMachineName();

    void state_machine_visualization(const ros::TimerEvent &);

    inline std::shared_ptr<SmaccStateInfo> getCurrentStateInfo() { return currentStateInfo_; }

    void publishTransition(const SmaccTransitionInfo &transitionInfo);

    /// this function should be implemented by the user to create the orthogonals
    virtual void onInitialize();

    bool getTransitionLogHistory(smacc_msgs::SmaccGetTransitionHistory::Request &req, smacc_msgs::SmaccGetTransitionHistory::Response &res);

    template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
    boost::signals2::connection createSignalConnection(TSmaccSignal &signal, TMemberFunctionPrototype callback, TSmaccObjectType *object);

    void disconnectSmaccSignalObject(void *object);

    template <typename StateType>
    void notifyOnStateEntryStart(StateType *state);

    template <typename StateType>
    void notifyOnStateEntryEnd(StateType *state);

    template <typename StateType>
    void notifyOnRuntimeConfigured(StateType *state);

    template <typename StateType>
    void notifyOnStateExitting(StateType *state);

    template <typename StateType>
    void notifyOnStateExited(StateType *state);

    template <typename StateType>
    void notifyOnRuntimeConfigurationFinished(StateType *state);

    inline uint64_t getCurrentStateCounter() const;

    inline ISmaccState *getCurrentState() const;

    inline const SmaccStateMachineInfo &getStateMachineInfo();

    template <typename InitialStateType>
    void buildStateMachineInfo();

    inline ros::NodeHandle getNode(){
        return nh_;
    };


protected:
    void checkStateMachineConsistence();

    void initializeROS(std::string smshortname);

    void onInitialized();

    template <typename TOrthogonal>
    void createOrthogonal();

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool getParam(std::string param_name, T &param_storage);

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    void setParam(std::string param_name, T param_val);

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool param(std::string param_name, T &param_val, const T &default_val) const;

    // The node handle for this state
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Timer timer_;
    ros::Publisher stateMachinePub_;
    ros::Publisher stateMachineStatusPub_;
    ros::Publisher transitionLogPub_;
    ros::ServiceServer transitionHistoryService_;

    // if it is null, you may be located in a transition. There is a small gap of time where internally
    // this currentState_ is null. This may change in the future.
    std::vector<ISmaccState*>  currentState_;

    std::shared_ptr<SmaccStateInfo> currentStateInfo_;

    smacc_msgs::SmaccStatus status_msg_;

    // orthogonals
    std::map<std::string, std::shared_ptr<smacc::ISmaccOrthogonal>> orthogonals_;

private:
    std::recursive_mutex m_mutex_;
    std::recursive_mutex eventQueueMutex_;

    StateMachineInternalAction stateMachineCurrentAction;

    std::map<void*, std::shared_ptr<CallbackCounterSemaphore>> stateCallbackConnections;

    // shared variables
    std::map<std::string, std::pair<std::function<std::string()>, boost::any>> globalData_;

    std::vector<smacc_msgs::SmaccTransitionLogEntry> transitionLogHistory_;

    smacc::SMRunMode runMode_;

    // Event to notify to the signaldetection thread that a request has been created...
    SignalDetector *signalDetector_;

    uint64_t stateSeqCounter_;

    void lockStateMachine(std::string msg);

    void unlockStateMachine(std::string msg);

    template <typename EventType>
    void propagateEventToStateReactors(ISmaccState *st, EventType *ev);

    std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo_;

    void updateStatusMessage();

    friend class ISmaccState;
    friend class SignalDetector;
};
} // namespace smacc

#include <smacc/impl/smacc_state_impl.h>
#include <smacc/impl/smacc_client_impl.h>
#include <smacc/impl/smacc_component_impl.h>
#include <smacc/impl/smacc_orthogonal_impl.h>
