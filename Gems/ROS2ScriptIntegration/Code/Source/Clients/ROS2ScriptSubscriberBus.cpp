#include <AzCore/EBus/EBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>
#include <ROS2ScriptIntegration/ROS2ScriptSubscriberBus.h>

namespace ROS2ScriptIntegration
{

    void SubscriberNotificationHandler::OnStdMsgBool(bool value)
    {
        Call(FN_OnStdMsgBool, value);
    }

    void SubscriberNotificationHandler::OnSensorMsgJoy(bool b0, bool b1, bool b2, bool b3, float f0, float f1, float f2, float f3)
    {
        Call(FN_OnSensorMsgJoy, b0, b1, b2, b3, f0, f1, f2, f3);
    }

    void SubscriberNotificationHandler::OnGeometryMsgPoseStamped(const AZStd::string frame, const AZ::Transform& transform)
    {
        Call(FN_OnGeometryMsgPoseStamped, frame, transform);
    }

    void SubscriberNotificationHandler::OnStdMsgString(const AZStd::string& value)
    {
        Call(FN_OnStdMsgString, value);
    }

    void SubscriberNotificationHandler::OnStdMsgFloat32(const float value)
    {
        Call(FN_OnStdMsgFloat32, value);
    }

    void SubscriberNotificationHandler::OnStdMsgUInt32(const uint32_t value)
    {
        Call(FN_OnStdMsgUInt32, value);
    }

    void SubscriberNotificationHandler::OnStdMsgInt32(const int32_t value)
    {
        Call(FN_OnStdMsgInt32, value);
    }

    void SubscriberNotificationHandler::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<SubscriberRequestBus>("SubscriberRequestBus")
                ->Attribute(AZ::Script::Attributes::Category, "ROS2")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "ROS2")
                ->Event("SubscribeToStdMsgBool", &SubscriberRequestBus::Events::SubscribeToStdMsgBool, { { { "Topic", "" } } })
                ->Event("SubscribeToSensorMsgJoy", &SubscriberRequestBus::Events::SubscribeToSensorMsgJoy, { { { "Topic", "" } } })
                ->Event(
                    "SubscribeToGeometryMsgPoseStamped",
                    &SubscriberRequestBus::Events::SubscribeToGeometryMsgPoseStamped,
                    { { { "Topic", "" } } })
                ->Event("SubscribeToString", &SubscriberRequestBus::Events::SubscribeToString, { { { "Topic", "" } } })
                ->Event("SubscribeToFloat32", &SubscriberRequestBus::Events::SubscribeToFloat32, { { { "Topic", "" } } })
                ->Event("SubscribeToUInt32", &SubscriberRequestBus::Events::SubscribeToUInt32, { { { "Topic", "" } } })
                ->Event("SubscribeToInt32", &SubscriberRequestBus::Events::SubscribeToInt32, { { { "Topic", "" } } });
        }
    }

    void SubscriberRequests::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<SubscriberNotificationsBus>("SubscriberNotificationsBus")
                ->Handler<SubscriberNotificationHandler>()
                ->Event("OnStdMsgBool", &SubscriberNotificationsBus::Events::OnStdMsgBool)
                ->Event(
                    "OnSensorMsgsJoy",
                    &SubscriberNotificationsBus::Events::OnSensorMsgJoy,
                    { { { "b0", "button 0" },
                        { "b1", "button 1" },
                        { "b2", "button 2" },
                        { "b3", "button 3" },
                        { "a0", "axis 0" },
                        { "a1", "axis 1" },
                        { "a2", "axis 2" },
                        { "a3", "axis 3" } } })
                ->Event(
                    "OnGeometryMsgPoseStamped",
                    &SubscriberNotificationsBus::Events::OnGeometryMsgPoseStamped,
                    { { { "frame", "button 0" } } })
                ->Event("OnStdMsgString", &SubscriberNotificationsBus::Events::OnStdMsgString)
                ->Event("OnStdMsgFloat32", &SubscriberNotificationsBus::Events::OnStdMsgFloat32)
                ->Event("OnStdMsgUInt32", &SubscriberNotificationsBus::Events::OnStdMsgUInt32)
                ->Event("OnStdMsgInt32", &SubscriberNotificationsBus::Events::OnStdMsgInt32);
        }
    }
} // namespace ROS2ScriptIntegration
