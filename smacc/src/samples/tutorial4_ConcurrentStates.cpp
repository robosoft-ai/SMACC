#include <ros/ros.h>
#include <smacc/smacc.h>

using namespace smacc;
using namespace std;
using namespace boost;

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

class Bas : public State {
public:
    Bas()
        : State({.outcomes = { "outcome3" } })
    {
    }

    virtual string execute(UserData& userdata) override
    {
        ROS_INFO("Executing state BAS");
        return "outcome3";
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smacc_example_state_machine");

    StateMachine sm_top({.outcomes = { "outcome4" } });

    sm_top.add<Bas>("BAS", {.transitions = { { "outcome3", "SUB" } } });

    auto sm_con = Concurrence::create(
        {.outcomes = { "outcome4", "outcome5" },
            .default_outcome = "outcome4",
            .outcome_map = { { "outcome5", { { "FOO", "outcome2" },
                                               { "BAR", "outcome5" } } } } });

    sm_con->add<Foo>("FOO");
    sm_con->add<Bar>("BAR");

    sm_top.add("CON", sm_con, {.transitions = { { "outcome4", "CON" }, { "outcome5", "outcome6" } } });

    auto outcome = sm_top.execute();
}
