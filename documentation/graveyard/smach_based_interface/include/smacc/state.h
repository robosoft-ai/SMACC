#pragma once
#include "state_node.h"
#include "utils.h"
#include <boost/any.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace smacc {
using namespace std;
using namespace boost;

///
/// \brief The StateInitializationParameters struct
///
struct StateInitializationParameters {
public:
    ///
    /// \brief outcomes
    ///
    vector<std::string> outcomes;
    vector<std::string> input_keys;
    vector<std::string> output_keys;
};

class State : public StateNode {
public:
    ///
    /// \brief State
    /// \param params
    ///
    State(StateInitializationParameters params);

    ///
    /// \brief execute
    /// \return
    ///
    virtual string execute();

    virtual string execute(UserData& userdata);
};
}
