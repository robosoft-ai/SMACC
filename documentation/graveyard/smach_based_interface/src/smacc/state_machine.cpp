#include <smacc/state_machine.h>

namespace smacc {

StateMachine::StateMachine(StateMachineInitializationParameters outcomes)
{
}

std::shared_ptr<StateMachine> StateMachine::create(StateMachineInitializationParameters outcomes)
{
    return std::shared_ptr<StateMachine>(new StateMachine(outcomes));
}

string StateMachine::execute()
{
}
}
