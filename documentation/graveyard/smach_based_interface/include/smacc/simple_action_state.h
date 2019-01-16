#pragma once
#include "state_node.h"
#include "utils.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace smacc {
using namespace std;
using namespace boost;

template <typename ActionType>
struct ActionStateInitializationParameters {
    typedef typename ActionType::_action_goal_type t;
    typedef typename t::_goal_type goaltype;

    goaltype goal;

    std::function<goaltype(UserData&)> goal_cb;
};

///
/// \brief The State class
///
template <typename ActionType>
class SimpleActionState : public StateNode {
public:
    using t = typename ActionType::_action_goal_type;
    using goaltype = typename t::_goal_type;

    static std::shared_ptr<SimpleActionState> create(string name)
    {
    }

    static std::shared_ptr<SimpleActionState> create(string name, ActionStateInitializationParameters<ActionType> params)
    {
    }

    static std::shared_ptr<SimpleActionState> create(string name, std::function<goaltype(UserData&)> params)
    {
    }
};
}
