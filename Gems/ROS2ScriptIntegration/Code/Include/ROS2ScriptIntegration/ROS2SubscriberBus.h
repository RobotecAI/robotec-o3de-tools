/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/string/string.h>
namespace ROS2ScriptIntegration
{

    class SubscriberNotifications : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(SubscriberNotifications, "{a64ebbc0-3c6e-44f5-8a58-ea921afa1c15}");
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        virtual OnStdMsgBool(const AZStd::string& topicName, bool value) = 0;
    };

    using SubscriberNotificationsBus = AZ::EBus<SubscriberNotifications>;

    //! This simple handler can be used for prototyping using LUA scripting
    class SubscriberNotificationHandler
        : public SubscriberNotifications::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(SubscriberNotificationHandler, "{cd4c7045-ea87-48be-9694-02a6e8a3c613}", AZ::SystemAllocator, JoyReceived);

        void OnStdMsgBool(const AZStd::string& topicName, bool value) override;
        static void Reflect(AZ::ReflectContext* context);
    };

} // namespace ROS2ScriptIntegration
