#pragma once
#include "state_node.h"
#include "utils.h"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace smacc {

///
/// \brief The CustomStateMachine class
///
class CustomStateMachine : public StateNode {
public:
    void changeState(string state)
    {
    }

    static std::shared_ptr<CustomStateMachine> loadPluginLib(string pathname)
    {
    }
};
}
