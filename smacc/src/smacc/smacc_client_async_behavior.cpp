#include <smacc/smacc_asynchronous_client_behavior.h>

namespace smacc
{
    void SmaccAsyncClientBehavior::executeOnEntry()
    {
        ISmaccClientBehavior::executeOnEntry();
    }

    void SmaccAsyncClientBehavior::executeOnExit()
    {
        ISmaccClientBehavior::executeOnExit();
    }

    SmaccAsyncClientBehavior::~SmaccAsyncClientBehavior()
    {

    }
} // namespace smacc