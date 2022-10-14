#include <smacc/smacc_asynchronous_client_behavior.h>

namespace smacc
{
    void SmaccAsyncClientBehavior::executeOnEntry()
    {
        ROS_INFO_STREAM("[" << getName() << "] Creating asynchronous onEntry thread");
        this->onEntryThread_ = std::async(std::launch::async,
                                          [=] {
                                              this->onEntry();
                                              this->postFinishEventFn_();
                                              return 0;
                                          });
    }

    void SmaccAsyncClientBehavior::executeOnExit()
    {
        ROS_INFO_STREAM("[" << getName() << "] onExit - join async onEntry thread");

        try
        {
            ros::Rate r(200);
            while (ros::ok())
            {
                bool valid = this->onEntryThread_.valid();
                if (valid)
                {
                    auto status = this->onEntryThread_.wait_for(std::chrono::milliseconds(20));
                    if (status == std::future_status::ready)
                    {
                        this->onEntryThread_.get();
                        break;
                    }
                }

                r.sleep();
                ros::spinOnce();
                ROS_DEBUG("waiting for finishing client behavior");
            }
        }
        catch (const std::exception &e)
        {
            ROS_DEBUG("[SmaccAsyncClientBehavior] trying to Join onEntry function, but it was already finished.");
        }

        ROS_INFO_STREAM("[" << getName() << "] onExit - Creating asynchronous onExit thread");
        this->onExitThread_ = std::async(std::launch::async,
                                         [=] {
                                             this->onExit();
                                             return 0;
                                         });
    }

    void SmaccAsyncClientBehavior::dispose()
    {
        ROS_DEBUG_STREAM("[" << getName() << "] Destroying client behavior- Waiting finishing of asynchronous onExit thread");
        try
        {
            this->onExitThread_.get();
        }
        catch (...)
        {
            ROS_DEBUG("[SmaccAsyncClientBehavior] trying to Join onExit function, but it was already finished.");
        }

        ROS_DEBUG_STREAM("[" << getName() << "] Destroying client behavior-  onExit thread finished. Proccedding destruction.");
    }

    SmaccAsyncClientBehavior::~SmaccAsyncClientBehavior()
    {
    }

    void SmaccAsyncClientBehavior::postSuccessEvent()
    {
        postSuccessEventFn_();
    }

    void SmaccAsyncClientBehavior::postFailureEvent()
    {
        postFailureEventFn_();
    }

} // namespace smacc
