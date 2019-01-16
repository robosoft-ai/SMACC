#include <smacc/state.h>

namespace smacc {

State::State(StateInitializationParameters params)
{
}

string State::execute()
{
    std::map<string, any> empty;
    this->execute(empty);
}

string State::execute(UserData& userdata)
{
}
}
