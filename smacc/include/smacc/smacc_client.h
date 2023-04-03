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
struct ComponentKey
{
    ComponentKey(const std::type_info *typeinfo, std::string name)
    {
        this->name = name;
        this->typeinfo = typeinfo;
        encodedKey = std::to_string((long)(void *)typeinfo) + "_" + name;
    }
    std::string encodedKey;
    const std::type_info *typeinfo;
    std::string name;

    bool operator<(const ComponentKey &other) const
    {
        return this->encodedKey < other.encodedKey;
    }
    bool operator==(const ComponentKey &other) const
    {
        return this->encodedKey == other.encodedKey;
    }
};

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

    template <typename TComponent>
    TComponent *getComponent(std::string name);

    template <typename TComponent>
    std::shared_ptr<TComponent> getComponentNew();

    template <typename TComponent>
    std::shared_ptr<TComponent> getComponentNew(std::string name);

    virtual smacc::introspection::TypeInfo::Ptr getType();

    inline ISmaccStateMachine *getStateMachine();

    template <typename TSmaccSignal, typename T>
    void connectSignal(TSmaccSignal &signal, void (T::*callback)(), T *object);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage);

    void getComponents(std::vector<std::shared_ptr<ISmaccComponent>> &components);

protected:

// it is called after the client initialization, provides information about the orthogonal it is located in
    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation() {}

    // components
    std::map<ComponentKey, std::shared_ptr<smacc::ISmaccComponent>> components_;

    template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
    SmaccComponentType *createComponent(TArgs... targs);

    template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
    SmaccComponentType *createNamedComponent(std::string name, TArgs... targs);

    // Assigns the owner of this resource to the given state machine parameter object
    void setStateMachine(ISmaccStateMachine *stateMachine);

    void setOrthogonal(ISmaccOrthogonal *orthogonal);

private:
    // A reference to the state machine object that owns this resource
    ISmaccStateMachine *stateMachine_;
    ISmaccOrthogonal *orthogonal_;

    friend class ISmaccOrthogonal;
    friend class ISmaccComponent;
};
} // namespace smacc
