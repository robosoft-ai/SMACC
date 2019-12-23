/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/common.h>
#include <smacc/component.h>
#include <typeinfo>

namespace smacc
{
class ISmaccClient
{
public:
    ISmaccClient();
    virtual ~ISmaccClient();

    virtual void initialize();

    // Assigns the owner of this resource to the given state machine parameter object
    void setStateMachine(ISmaccStateMachine *stateMachine);

    // Returns a custom identifier defined by the specific plugin implementation
    virtual std::string getName() const;

    template <typename EventType>
    void postEvent(const EventType &ev);

    template <typename EventType>
    void postEvent();

    template <typename TComponent>
    TComponent *getComponent();

    template <typename SmaccComponentType, typename... TArgs>
    SmaccComponentType *createComponent(TArgs... targs);

protected:
    // A reference to the state machine object that owns this resource
    ISmaccStateMachine *stateMachine_;

    // components
    std::map<const std::type_info *, std::shared_ptr<smacc::ISmaccComponent>> components_;

private:
    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes() {}

    friend class Orthogonal;
};
} // namespace smacc