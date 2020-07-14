/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/common.h>

namespace smacc
{
    class ISmaccState;

    class ISmaccClientBehavior
    {
    public:
        ISmaccClientBehavior();

        virtual ~ISmaccClientBehavior();

        inline ISmaccStateMachine *getStateMachine();

        std::string getName() const;
        
        template <typename SmaccClientType>
        void requiresClient(SmaccClientType *&storage);

        template <typename SmaccComponentType>
        void requiresComponent(SmaccComponentType *&storage);

    protected:
        virtual void runtimeConfigure();

        virtual void onEntry() = 0;

        virtual void onExit() = 0;

        template <typename EventType>
        void postEvent(const EventType &ev);

        template <typename EventType>
        void postEvent();

        inline ISmaccState *getCurrentState();

    private:
        template <typename TObjectTag, typename TDerived>
        void configureEventSourceTypes();

        // a reference to the owner state machine
        ISmaccStateMachine *stateMachine_;

        // a reference to the state where the client behavior is being executed
        ISmaccState *currentState;

        smacc::ISmaccOrthogonal *currentOrthogonal;

        friend class ISmaccState;
        friend class ISmaccOrthogonal;
    };
} // namespace smacc