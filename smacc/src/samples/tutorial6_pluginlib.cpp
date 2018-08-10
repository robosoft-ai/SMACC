#include <smacc/TestAction.h>
#include <smacc/smacc.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <chrono>
#include <future>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <thread>

using namespace smacc;
using namespace std;
using namespace boost;
using namespace std::chrono;

///
/// \brief The AdhocStateMachine class: example of custom state machine of some product independent to smacc and ROS
///
class AdhocStateMachine {
public:
    enum class State { RUNNING,
        PAUSED,
        FINISHED };

    State currentState;
    std::chrono::time_point<std::chrono::system_clock> startTime;
    int timeMultiplier;
    int result;

    void Initialize(int time_multiplier)
    {
        currentState = State::RUNNING;
        this->startTime = high_resolution_clock::now();
    }

    void Pause()
    {
        currentState = State::PAUSED;
    }

    void Finish()
    {
        currentState = State::FINISHED;
    }

    int Run()
    {
        while (Update()) {
        }

        return this->result;
    }

    bool Update()
    {
        std::this_thread::sleep_for(milliseconds(1000));
        auto now = high_resolution_clock::now();
        auto deltatime = now - startTime;

        if (deltatime > timeMultiplier * milliseconds(10000)) {
            currentState = State::FINISHED;
            result = 10;
            return false;
        }

        return true;
    }
};

///
/// \brief The code that is needed to implement to create the rospluginlib state machine.
/// It will run in a different thread
///
class CustomStateMachineAdapter : public CustomStateMachine {
    //dependent code to the final implementation

    AdhocStateMachine sm;

    void onStart(UserData& userdata)
    {
        auto timemultiplier = any_cast<int>(userdata["timemultiplier"]);
        this->changeState("Paused");
        sm.Initialize(timemultiplier);

        auto result = std::async([&, this] {
            auto res = this->sm.Run();
            this->changeState("Finished");
            userdata["result"] = res;
        });
    }
};

PLUGINLIB_EXPORT_CLASS(CustomStateMachineAdapter, smacc::CustomStateMachine);

class Foo : public State {
public:
    int counter_;
    Foo()
        : State({.outcomes = { "outcome1", "outcome2" } })
    {
        counter_ = 0;
    }

    virtual string execute(UserData& userData) override
    {
        ROS_INFO("Executing state FOO");
        if (counter_ < 3) {
            counter_ += 1;
            userData["timemultiplier"] = counter_;
            return "toCustomStateMachineTransition";
        } else {
            return "endSM";
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smacc_example_state_machine");

    StateMachine sm({.outcomes = { "outcome4" } });

    sm.add<Foo>("FOO", {.transitions = { { "endSM", "outcome4" },
                            { "repeatThisStateTransition", "FOO" } } });

    auto puglinstatemachine = CustomStateMachine::loadPluginLib("path/to/statemachine.so");

    sm.add("CUSTOM", puglinstatemachine, {.transitions = { { "finished", "FOO" } } });

    auto outcome = sm.execute();
}
