#include <AzCore/EBus/EBus.h>
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>

#include <ROS2ScriptIntegration/ROS2ScriptPublisherBus.h>

namespace ROS2ScriptIntegration
{

    void PublisherRequests::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<PublisherRequestBus>("PublisherRequestBus")
                ->Attribute(AZ::Script::Attributes::Category, "ROS2")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "ROS2")
                ->Event(
                    "PublishStdMsgsString", &PublisherRequestBus::Events::PublishStdMsgsString, { { { "Topic", "" }, { "Value", "" } } })
                ->Event("PublishStdMsgEmpty", &PublisherRequestBus::Events::PublishStdMsgEmpty, { { { "Topic", "" } } })
                ->Event("PublishStdMsgUInt32", &PublisherRequestBus::Events::PublishStdMsgUInt32, { { { "Topic", "" }, { "Value", "" } } })
                ->Event("PublishStdMsgInt32", &PublisherRequestBus::Events::PublishStdMsgInt32, { { { "Topic", "" }, { "Value", "" } } })
                ->Event(
                    "PublishStdMsgFloat32", &PublisherRequestBus::Events::PublishStdMsgFloat32, { { { "Topic", "" }, { "Value", "" } } })
                ->Event("PublishStdMsgBool", &PublisherRequestBus::Events::PublishStdMsgBool, { { { "Topic", "" }, { "Value", "" } } })
                ->Event(
                    "PublishGeometryMsgsTwist",
                    &PublisherRequestBus::Events::PublishGeometryMsgsTwist,
                    { { { "Topic", "" }, { "Linear", "" }, { "Angular", "" } } })
                ->Event(
                    "PublishGeometryMsgTransform",
                    &PublisherRequestBus::Events::PublishGeometryMsgTransform,
                    { { { "Topic", "" }, { "Transform", "" } } })
                ->Event(
                    "PublishGeometryMsgVector3",
                    &PublisherRequestBus::Events::PublishGeometryMsgVector3,
                    { { { "Topic", "" }, { "Vector", "" } } })
                ->Event(
                    "PublishGeometryMsgQuaternion",
                    &PublisherRequestBus::Events::PublishGeometryMsgQuaternion,
                    { { { "Topic", "" }, { "Quaternion", "" } } })
                ->Event(
                    "PublishGeometryMsgPoint32",
                    &PublisherRequestBus::Events::PublishGeometryMsgPoint32,
                    { { { "Topic", "" }, { "Point", "" } } })
                ->Event(
                    "PublishGeometryMsgPoseStamped",
                    &PublisherRequestBus::Events::PublishGeometryMsgPoseStamped,
                    { { { "Topic", "" }, { "Frame", "" }, { "Transform", "" } } });
        }
    }
} // namespace ROS2ScriptIntegration
