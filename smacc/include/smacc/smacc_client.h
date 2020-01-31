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

    // Returns a custom identifier defined by the specific plugin implementation
    virtual std::string getName() const;

    template <typename EventType>
    void postEvent(const EventType &ev);

    template <typename EventType>
    void postEvent();

    template <typename TComponent>
    TComponent *getComponent();

    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes() {}

    virtual smacc::introspection::TypeInfo::Ptr getType();

    inline ISmaccStateMachine *getStateMachine();

    template <typename TSmaccSignal, typename T>
    void connectSignal(TSmaccSignal &signal, void (T::*callback)(), T *object);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage);

protected:
    // components
    std::map<const std::type_info *, std::shared_ptr<smacc::ISmaccComponent>> components_;

    template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
    SmaccComponentType *createComponent(TArgs... targs);

    // Assigns the owner of this resource to the given state machine parameter object
    void setStateMachine(ISmaccStateMachine *stateMachine);

    void setOrthogonal(ISmaccOrthogonal* orthogonal);

private:
    // A reference to the state machine object that owns this resource
    ISmaccStateMachine *stateMachine_;
    ISmaccOrthogonal *orthogonal_;

    friend class ISmaccOrthogonal;
};
} // namespace smacc