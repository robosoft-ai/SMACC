#pragma once
#include "concurrence.h"
#include "simple_action_state.h"
#include "state_node.h"
#include "utils.h"

#include <map>
#include <memory>
#include <string>

namespace smacc {
using namespace std;

///
/// \brief The StateMachineInitializationParameters struct
///
struct StateMachineInitializationParameters {
    ///
    /// \brief outcomes
    ///
    vector<std::string> outcomes;
};

///
/// \brief The StateMachine class
///
class StateMachine : public StateNode {
public:
    StateMachine(StateMachineInitializationParameters outcomes);

    static std::shared_ptr<StateMachine> create(StateMachineInitializationParameters outcomes);

    template <typename StateNodeType>
    void add(string stateName, std::shared_ptr<StateNodeType> stateMachine, AddStateParameters params)
    {
    }

    template <typename StateType>
    void add(string stateName, AddStateParameters params)
    {
        this->add(stateName, std::make_shared<StateType>(), params);
    }

    template <typename ActionType>
    void addActionState(string stateName, std::shared_ptr<SimpleActionState<ActionType> > state, map<string, string> mapping)
    {
    }

    ///
    /// \brief execute
    /// \return
    ///
    string execute();
};
}
