#include <conditional/sb_conditional.h>

namespace smacc
{
namespace state_reactors
{
SbConditional::~SbConditional()
{
}

bool SbConditional::triggers()
{
    return this->conditionFlag;
}
} // namespace state_reactors
} // namespace smacc