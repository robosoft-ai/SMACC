#pragma once
#include "state.h"
#include "utils.h"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace smacc {
using namespace std;

struct ConcurrenceInitializationParameters {
    ///
    /// \brief outcomes
    ///
    vector<std::string> outcomes;

    string default_outcome;

    map<string, map<string, string> > outcome_map;
};

///
/// \brief The Concurrence class
///
class Concurrence : public StateNode {
public:
    static std::shared_ptr<Concurrence> create(ConcurrenceInitializationParameters params);

    ///
    /// \brief add
    /// \param stateName
    /// \param state
    ///
    void add(string stateName, std::shared_ptr<State> state)
    {
    }

    template <typename StateType>
    void add(string stateName)
    {
        this->add(stateName, std::make_shared<StateType>());
    }
};
}
