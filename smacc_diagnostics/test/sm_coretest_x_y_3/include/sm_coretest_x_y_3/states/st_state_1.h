#include <smacc/smacc.h>
#define iterationCheck  10000

namespace sm_coretest_x_y_3
{

extern int counter;
extern ros::Time startTime;

// STATE DECLARATION
struct State1 : smacc::SmaccState<State1, SmCoreTestXY3>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<AutomaticTransitionEvent, State2, SUCCESS>>
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
    }

    void onEntry()
    {
        counter++;
                
        if (counter > iterationCheck )
        {
            ROS_ERROR("ey");
            auto now = ros::Time::now();;
            auto ellapsed =  now - startTime ;
            auto freqHz = iterationCheck / ellapsed.toSec();

            startTime = now;
            while(ros::ok())
            {
                ros::Duration(0.2).sleep();
                ROS_ERROR("A %d iterations in %lf seconds. Ferquency: %lf Hz", iterationCheck,  ellapsed.toSec(), freqHz);
            }
        }

        this->postEvent<AutomaticTransitionEvent>();
    }
};
} // namespace sm_coretest_x_y_3