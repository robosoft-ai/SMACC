#pragma once

#include <smacc/client.h>

namespace smacc
{
template <typename ServiceType>
class SmaccServiceClient : public smacc::ISmaccClient
{
    public:
    SmaccServiceClient()
    {
        initialized_=false;
    }

    void initialize(std::string serviceName)
    {
        if (!initialized_)
        {
            ROS_INFO_STREAM("[" << this->getName() << "] Client Service: " << serviceName);
            this->initialized_ = true;

            client_ = nh_.serviceClient<ServiceType>(serviceName);
        }
    }

    bool call(ServiceType  &srvreq)
    {
        return client_.call(srvreq);
    }

protected:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    bool initialized_;
};
} // namespace smacc