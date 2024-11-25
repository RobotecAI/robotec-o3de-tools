#include "SplineSubscriber.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <ROS2/Georeference/GeoreferenceBus.h>

namespace SplineTools
{

    void SplineSubscriber::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void SplineSubscriber::Reflect(AZ::ReflectContext* context)
    {
        SplineSubscriberConfiguration::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineSubscriber, AZ::Component>()->Version(0)->Field("m_config", &SplineSubscriber::m_config);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SplineSubscriber>("SplineSubscriber", "Configuration for the SplineSubscriber component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "SplineSubscriber")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineSubscriber::m_config,
                        "Configuration",
                        "Configuration for the SplineSubscriber component");
            }
        }
    }

    void SplineSubscriber::Activate()
    {
        if (m_config.m_resetOnActivation)
        {
            LmbrCentral::SplineComponentRequestBus::Event(GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::ClearVertices);
        }
        auto node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(node, "ROS 2 Node is not available");
        m_subscription = node->create_subscription<nav_msgs::msg::Path>(
            m_config.m_topic.m_topic.data(),
            m_config.m_topic.GetQoS(),
            [this](const nav_msgs::msg::Path::SharedPtr msg)
            {
                OnSplineReceived(*msg);
            });
    }

    bool SplineSubscriber::GetOffsetTransform(AZ::Transform& transform)
    {
        AZ::EBusAggregateResults<AZ::EntityId> aggregator;
        const LmbrCentral::Tag tag = AZ::Crc32(m_config.m_startOffsetTag);
        LmbrCentral::TagGlobalRequestBus::EventResult(aggregator, tag, &LmbrCentral::TagGlobalRequests::RequestTaggedEntities);

        AZ_Warning(
            "SplineSubscriber",
            aggregator.values.size() <= 1,
            "Multiple entities found with tag %s. The first entity will be used.",
            m_config.m_startOffsetTag.c_str());

        AZ_Warning("SplineSubscriber", !aggregator.values.empty(), "No entity with tag found %s.", m_config.m_startOffsetTag.c_str());

        if (!aggregator.values.empty())
        {
            AZ::Transform offsetTransform = AZ::Transform::CreateIdentity();
            const AZ::EntityId& entityId = aggregator.values[0];
            AZ::TransformBus::EventResult(offsetTransform, entityId, &AZ::TransformBus::Events::GetWorldTM);
            transform = offsetTransform;
            return true;
        }

        return false;
    }

    void SplineSubscriber::OnSplineReceived(const nav_msgs::msg::Path& msg)
    {
        AZ::SplinePtr splinePtr;
        LmbrCentral::SplineComponentRequestBus::EventResult(
            splinePtr, GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::GetSpline);
        AZ_Assert(splinePtr, "SplineComponentRequestBus::Events::GetSpline failed");

        AZ::Transform worldTm = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTm, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        worldTm.Invert();

        AZ::Transform offsetTransform = AZ::Transform::CreateIdentity();
        auto hasOffset = GetOffsetTransform(offsetTransform);

        AZStd::string frame{ msg.header.frame_id.c_str(), msg.header.frame_id.size() };
        AZStd::to_upper(frame.begin(), frame.end());
        AZ_Printf("SplineSubscriber", "Frame: %s", frame.data());
        AZStd::vector<AZ::Vector3> points(msg.poses.size());
        for (size_t i = 0; i < msg.poses.size(); ++i)
        {
            const auto& poseStamped = msg.poses[i];
            const auto& pose = poseStamped.pose;
            const AZ::Vector3 posePoint = AZ::Vector3(pose.position.x, pose.position.y, pose.position.z);

            if (frame.empty())
            {
                if (hasOffset)
                {
                    auto referenceTranslation = offsetTransform.TransformPoint(posePoint);
                    points[i] = worldTm.TransformPoint(referenceTranslation);
                }
                else
                {
                    points[i] = worldTm.TransformPoint(posePoint);
                }
            }
            else if (frame == "LOCAL")
            {
                points[i] = posePoint;
            }
            else if (frame == "WGS84" && m_config.m_allowWGS84)
            {
                ROS2::WGS::WGS84Coordinate currentPositionWGS84;
                currentPositionWGS84.m_latitude = pose.position.x;
                currentPositionWGS84.m_longitude = pose.position.y;
                currentPositionWGS84.m_altitude = pose.position.z;
                AZ::Vector3 levelPosition{ 0 };
                ROS2::GeoreferenceRequestsBus::BroadcastResult(
                    levelPosition, &ROS2::GeoreferenceRequests::ConvertFromWGS84ToLevel, currentPositionWGS84);
                points[i] = worldTm.TransformPoint(levelPosition);
            }
            else
            {
                AZ_Error("SplineSubscriber", false, "Not implemented with frame %s", frame.data());
            }
        }
        LmbrCentral::SplineComponentRequestBus::Event(GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::ClearVertices);
        LmbrCentral::SplineComponentRequestBus::Event(GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::SetVertices, points);
    }

    void SplineSubscriber::Deactivate()
    {
        m_subscription.reset();
    }

} // namespace SplineTools