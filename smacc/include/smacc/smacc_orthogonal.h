/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/common.h>
#include <utility>

namespace smacc
{

class IOrthogonal
{
public:
    void setStateMachine(ISmaccStateMachine *value);

    void addClientBehavior(std::shared_ptr<smacc::SmaccClientBehavior> statereactor);

    void onEntry();

    void onExit();

    virtual std::string getName() const;

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage);

    inline const std::vector<std::shared_ptr<smacc::ISmaccClient>> &getClients()
    {
        return clients_;
    }

    inline const std::vector<std::shared_ptr<smacc::SmaccClientBehavior>> &getClientBehaviors() const
    {
        return this->clientBehaviors_;
    }

    template <typename TClientBehavior>
    TClientBehavior *getClientBehavior();

protected:
    virtual void onInitialize();

    ISmaccStateMachine *stateMachine_;

    std::vector<std::shared_ptr<smacc::SmaccClientBehavior>> clientBehaviors_;

    std::vector<std::shared_ptr<smacc::ISmaccClient>> clients_;
};

} // namespace smacc