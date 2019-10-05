#pragma once

#include <smacc/client.h>
#include <boost/optional/optional_io.hpp>

namespace smacc
{
template <typename ServiceType>
class SmaccServiceClient : public smacc::ISmaccClient
{
public:
    boost::optional<std::string> serviceName_;

    SmaccServiceClient()
    {
        initialized_ = false;
    }

    virtual void initialize() override
    {
        if (!initialized_)
        {
            if (!serviceName_)
            {
                ROS_ERROR("service client with no service name set. Skipping.");
            }
            else
            {
                ROS_INFO_STREAM("[" << this->getName() << "] Client Service: " << *serviceName_);
                this->initialized_ = true;

                client_ = nh_.serviceClient<ServiceType>(*serviceName_);
            }
        }
    }

    bool call(ServiceType &srvreq)
    {
        return client_.call(srvreq);
    }

protected:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    bool initialized_;
};
} // namespace smacc