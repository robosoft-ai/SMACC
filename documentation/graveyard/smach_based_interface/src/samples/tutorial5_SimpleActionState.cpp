#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
/*
#include <ros/ros.h>
#include <smacc/TestAction.h>
#include <smacc/smacc.h>

using namespace smacc;
using namespace std;
using namespace boost;

class TestServer {
public:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<smacc::TestAction> sas;
    TestServer(string name)
        : sas(nh, name, boost::bind(&TestServer::executeCB, this, _1), false)
    {
        sas.start();
    }

    void executeCB(const smacc::TestGoalConstPtr& msg)
    {
        if (msg->goal == 0)
            sas.setSucceeded();
        else if (msg->goal == 1)
            sas.setAborted();
        else if (msg->goal == 2)
            sas.setPreempted();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smacc_example_actionlib");

    TestServer server("test_action");

    StateMachine sm0({.outcomes = { "succedeed", "aborted", "preempted" } });

    auto goal_default_state = SimpleActionState<TestAction>::create("test_action");

    TestGoal testgoal;
    testgoal.goal = 1;

    auto goal_static_state = SimpleActionState<TestAction>::create("test_action", {.goal = testgoal });

    auto goal_cb =
        [](const UserData&) -> TestGoal {
        auto goal = TestGoal();
        goal.goal = 2;
        return goal;
    };

    auto goal_callback_state = SimpleActionState<TestAction>::create("test_action", goal_cb);

    sm0.addActionState("GOAL_DEFAULT", goal_default_state, { { "succeeded", "GOAL_STATIC" } });

    sm0.addActionState("GOAL_STATIC", goal_static_state, { { "aborted", "GOAL_CB" } });

    sm0.addActionState("GOAL_CB", goal_callback_state, { { "aborted", "succeeded" } });

    auto outcome = sm0.execute();
}*/
