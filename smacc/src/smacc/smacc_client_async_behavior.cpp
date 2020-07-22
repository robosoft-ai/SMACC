#include <smacc/smacc_asynchronous_client_behavior.h>

namespace smacc
{
    void SmaccAsyncClientBehavior::executeOnEntry()
    {
        ROS_INFO_STREAM("[" << getName() << "] Creating asynchronous onEntry thread");
        this->onEntryThread_ = std::make_shared<std::thread>(std::thread(
            [=] {
                this->onEntry();
            }));
    }

    void SmaccAsyncClientBehavior::executeOnExit()
    {
        ISmaccClientBehavior::executeOnExit();
        ROS_INFO_STREAM("[" << getName() << "] onExit - Waiting finishing of asynchronous onEntry thread");
        this->onEntryThread_->join();
        ROS_INFO_STREAM("[" << getName() << "] onExit - Creating asynchronous onExit thread");
        this->onExitThread_ = std::make_shared<std::thread>(
            [=] {
                this->onExit();
            });
    }

    SmaccAsyncClientBehavior::~SmaccAsyncClientBehavior()
    {
        ROS_DEBUG_STREAM("[" << getName() << "] Destroying client behavior- Waiting finishing of asynchronous onExit thread");
        this->onExitThread_->join();
        ROS_DEBUG_STREAM("[" << getName() << "] Destroying client behavior-  onExit thread finished. Proccedding destruction.");
    }
} // namespace smacc
