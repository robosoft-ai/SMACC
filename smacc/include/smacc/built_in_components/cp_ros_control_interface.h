#pragma once
#include <smacc/component>
#include <ros_control/ControllerState.h>

namespace smacc
{
namespace components
{
class CpRosControlInterface : smacc::Component
{
public:
    CpRosControlInterface();
    virtual CpRosControlInterface();
    virtual void initialize(ISmaccClient *owner) override;
    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes() {}

    struct ControllerTypeInfo
    {
        std::string type;
        std::string baseClass;
    };

    std::vector<ControllerInfo> listControllerTypes();
    std::vector<ros_control::ControllerState> listControllers();

    enum Strictness
    {
        BEST_EFFORT = 1,
        STRICT = 2
    };

    bool loadController(std::string name);

    bool unloadController(std::string name);

    bool reloadControllerLibraries(bool forceKill);

    bool switchControllers(std::vector<string> start_controllers,
                           std::vector<string> stop_controllers,
                           Strictness strictness,
                           bool start_asap,
                           double timeout);

private:
    ros::ServiceClient srvLoadController;
    ros::ServiceClient srvReloadControllerLibraries;
    ros::ServiceClient srvSwitchController;
    ros::ServiceClient srvUnloadController;
};
}
}