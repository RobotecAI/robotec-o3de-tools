#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/std/string/string.h>

namespace ROS2ScriptIntegration
{

    //! This bus is used to subscribe to ROS2 topics from LUA/ScriptCanvas scripts.
    //! It is not recommended to use this bus for C++ components, since this bus is limited to simple messages.
    //! To add new message types, new functions:
    //!      - a virtual function in the SubscriberRequests class in SubscriberRequests
    //!      - a reflection in the SubscriberRequests::Reflect function
    //!      - an implementation in the ROS2ScriptIntegrationSystemComponent class in ROS2ScriptIntegrationSystemComponent.h
    //!        that creates a new subscriber.
    //!      - a new function in the SubscriberNotifications class in SubscriberNotifications
    //!      - a reflection in the SubscriberNotifications::Reflect function
    //!      - add a new function name in AZ_EBUS_BEHAVIOR_BINDER for the SubscriberNotificationHandler
    //!      - an implementation in SubscriberNotificationHandler that calls the new function

    class SubscriberRequests : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(SubscriberRequests, "{71935101-17de-4636-97f8-dea68938706d}");
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        virtual void SubscribeToStdMsgBool(const AZStd::string& topicName) = 0;
        virtual void SubscribeToSensorMsgJoy(const AZStd::string& topicName) = 0;
        virtual void SubscribeToGeometryMsgPoseStamped(const AZStd::string& topicName) = 0;
        virtual void SubscribeToString(const AZStd::string& topicName) = 0;
        virtual void SubscribeToFloat32(const AZStd::string& topicName) = 0;
        virtual void SubscribeToUInt32(const AZStd::string& topicName) = 0;
        virtual void SubscribeToInt32(const AZStd::string& topicName) = 0;
        virtual void SubscribeToVector3(const AZStd::string& topicName) = 0;

        static void Reflect(AZ::ReflectContext* context);
    };

    using SubscriberRequestBus = AZ::EBus<SubscriberRequests>;
    using SubscriberInterface = AZ::Interface<SubscriberRequests>;

    class SubscriberNotifications : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(SubscriberNotifications, "{a64ebbc0-3c6e-44f5-8a58-ea921afa1c15}");
        using BusIdType = AZStd::string;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        virtual void OnStdMsgBool(bool value) = 0;
        virtual void OnSensorMsgJoy(bool b0, bool b1, bool b2, bool b3, float f0, float f1, float f2, float f3) = 0;
        virtual void OnGeometryMsgPoseStamped(const AZStd::string frame, const AZ::Transform& transform) = 0;
        virtual void OnStdMsgString(const AZStd::string& value) = 0;
        virtual void OnStdMsgFloat32(const float value) = 0;
        virtual void OnStdMsgUInt32(const uint32_t value) = 0;
        virtual void OnStdMsgInt32(const int32_t value) = 0;
        virtual void OnGeometryMsgVector3(const AZ::Vector3& value) = 0;
    };

    using SubscriberNotificationsBus = AZ::EBus<SubscriberNotifications>;
    //! This simple handler can be used for prototyping using LUA scripting
    class SubscriberNotificationHandler
        : public SubscriberNotificationsBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            SubscriberNotificationHandler,
            "{cd4c7045-ea87-48be-9694-02a6e8a3c613}",
            AZ::SystemAllocator,
            OnStdMsgBool,
            OnSensorMsgJoy,
            OnGeometryMsgPoseStamped,
            OnStdMsgString,
            OnStdMsgFloat32,
            OnStdMsgUInt32,
            OnStdMsgInt32,
            OnGeometryMsgVector3);

        void OnStdMsgBool(bool value) override;
        void OnSensorMsgJoy(bool b0, bool b1, bool b2, bool b3, float f0, float f1, float f2, float f3) override;
        void OnGeometryMsgPoseStamped(const AZStd::string frame, const AZ::Transform& transform) override;
        void OnStdMsgString(const AZStd::string& value) override;
        void OnStdMsgFloat32(const float value) override;
        void OnStdMsgUInt32(const uint32_t value) override;
        void OnStdMsgInt32(const int32_t value) override;
        void OnGeometryMsgVector3(const AZ::Vector3& value) override;

        static void Reflect(AZ::ReflectContext* context);
    };

} // namespace ROS2ScriptIntegration
