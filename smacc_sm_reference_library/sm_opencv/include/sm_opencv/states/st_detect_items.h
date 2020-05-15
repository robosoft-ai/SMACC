#include <smacc/smacc.h>
namespace sm_opencv
{
    // STATE DECLARATION
    struct StDetectItems : smacc::SmaccState<StDetectItems, SmOpenCV>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<> reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
        }
    };
} // namespace sm_opencv