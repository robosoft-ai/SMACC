#include <conditional/sr_conditional.h>

namespace smacc
{
namespace state_reactors
{
SrConditional::~SrConditional()
{
}

bool SrConditional::triggers()
{
    return this->conditionFlag;
}
} // namespace state_reactors
} // namespace smacc