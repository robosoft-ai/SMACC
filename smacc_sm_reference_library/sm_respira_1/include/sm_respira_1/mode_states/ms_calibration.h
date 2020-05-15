#include <smacc/smacc.h>
namespace sm_respira_1
{
// STATE DECLARATION
class MsCalibration : public smacc::SmaccState<MsCalibration, SmRespira1, StCalibrationStep1, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_respira_1