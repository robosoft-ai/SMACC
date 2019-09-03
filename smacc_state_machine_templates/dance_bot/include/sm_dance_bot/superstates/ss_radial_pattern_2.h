namespace SS2
{
//forward declaration for initial ssr
class ssr_radial_rotate;

struct ss_radial_pattern_2 : smacc::SmaccState<ss_radial_pattern_2, sm_dance_bot, ssr_radial_rotate>
{
  using SmaccState::SmaccState;
};

//forward declaration for the superstate
using SS = ss_radial_pattern_2;
#include <sm_dance_bot/superstate_routines/ssr_radial_end_point.h>
#include <sm_dance_bot/superstate_routines/ssr_radial_return.h>
#include <sm_dance_bot/superstate_routines/ssr_radial_rotate.h>
} // namespace SS2