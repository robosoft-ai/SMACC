#include <conditional/sb_conditional.h>

namespace smacc
{
namespace state_behaviors
{
SbConditional::~SbConditional()
{
}

bool SbConditional::triggers()
{
    return this->conditionFlag;
}
} // namespace state_behaviors
} // namespace smacc