#include <ros/ros.h>
#include <smacc/smacc.h>

using namespace smacc;
using namespace std;
using namespace boost;

class Foo : public State {
public:
    int counter_;
    Foo()
        : State({.outcomes = { "outcome1", "outcome2" },
              .input_keys = { "foo_counter_in" },
              .output_keys = { "foo_counter_out" } })
    {
        counter_ = 0;
    }

    virtual string execute(map<string, any>& userdata) override
    {
        ROS_INFO("Executing state FOO");

        float foo_counter_in = any_cast<int>(userdata["foo_counter_in"]);

        if (foo_counter_in < 3.0) {
            userdata["outcome1"] = foo_counter_in + 1.0F;
            return "outcome1";
        } else {
            return "outcome2";
        }
    }
};

class Bar : public State {
public:
    Bar()
        : State({.outcomes = { "outcome1" },
              .input_keys = { "bar_counter_in" } })
    {
    }

    virtual string execute(UserData& userdata) override
    {
        ROS_INFO("Executing state BAR");
        ROS_INFO("Counter = %f", any_cast<float>(userdata["bar_counter_in"]));
        return "outcome1";
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smacc_example_state_machine");

    StateMachine sm({.outcomes = { "outcome4" } });

    sm.add<Foo>("FOO", {.transitions = { { "outcome1", "BAR" },
                            { "outcome2", "outcome4" } },
                           .remapping = { { "foo_counter_in", "sm_counter" },
                               { "foo_counter_out", "sm_counter" } } });

    sm.add<Bar>("BAR", {.transitions = { { "outcome1", "FOO" } },
                           .remapping = { { "bar_counter_in", "sm_counter" } } });

    auto outcome = sm.execute();
}
