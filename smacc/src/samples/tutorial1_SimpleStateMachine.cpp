#include <ros/ros.h>
#include <smacc/smacc.h>

using namespace smacc;
using namespace std;

class Foo : public State {
public:
    int counter_;
    Foo()
        : State({.outcomes = { "outcome1", "outcome2" } })
    {
        counter_ = 0;
    }

    virtual string execute() override
    {
        ROS_INFO("Executing state FOO");
        if (counter_ < 3) {
            counter_ += 1;
            return "outcome1";
        } else {
            return "outcome2";
        }
    }
};

class Bar : public State {
public:
    Bar()
        : State({.outcomes = { "outcome1" } })
    {
    }

    virtual string execute() override
    {
        ROS_INFO("Executing state BAR");
        return "outcome1";
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smacc_example_state_machine");

    StateMachine sm({.outcomes = { "outcome4" } });

    sm.add<Foo>("FOO", {.transitions = { { "outcome1", "BAR" },
                            { "outcome2", "outcome4" } } });

    sm.add<Bar>("BAR", {.transitions = {
                            { "outcome1", "FOO" } } });

    auto outcome = sm.execute();
}
