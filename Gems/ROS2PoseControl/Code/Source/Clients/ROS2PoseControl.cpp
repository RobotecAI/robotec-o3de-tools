#include "ROS2PoseControl.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Script/ScriptTimePoint.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <imgui/imgui.h>

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2PoseControl/ROS2PoseControlConfiguration.h>
#include <tf2_ros/transform_listener.h>

#include <AzCore/std/string/regex.h>
#include <AzFramework/Entity/GameEntityContextComponent.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <RigidBodyComponent.h>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>

namespace ROS2PoseControl
{
    ROS2PoseControl::ROS2PoseControl()
    {
        m_configuration.m_poseTopicConfiguration.m_topic = "goal_pose";
        m_configuration.m_poseTopicConfiguration.m_type = "geometry_msgs::msg::PoseStamped";
        m_configuration.m_targetFrame = "base_link";
        m_configuration.m_referenceFrame = "map";
    }

    void ROS2PoseControl::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ROS2PoseControl::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "ROS2PoseControl requires a valid ROS 2 node.")
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(ros2Node->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        if (m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::TF2)
        {
            AZ::TickBus::Handler::BusConnect();
        }
        else if (m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::PoseMessages)
        {
            OnTopicConfigurationChanged();
        }
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();

        AZ::TickBus::QueueFunction(
            [this]()
            {
                ApplyTransform(AZ::Transform::CreateIdentity());
            });
    }

    void ROS2PoseControl::Deactivate()
    {
        m_poseSubscription.reset();
        m_tf_buffer.reset();
        m_tf_listener.reset();
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void ROS2PoseControl::DisablePhysics()
    {
        if (auto* rigidBodyComponent = m_entity->FindComponent<PhysX::RigidBodyComponent>())
        {
            rigidBodyComponent->DisablePhysics();
        }
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, GetEntityId(), &AZ::TransformBus::Events::GetAllDescendants);
        for (const AZ::EntityId& child : children)
        {
            AZ::Entity* childEntity;
            AZ::ComponentApplicationBus::BroadcastResult(childEntity, &AZ::ComponentApplicationRequests::FindEntity, child);
            if (auto* childRigidBodyComponent = childEntity->FindComponent<PhysX::RigidBodyComponent>())
            {
                childRigidBodyComponent->DisablePhysics();
            }
        }
    }

    AZ::Outcome<AZ::Transform, AZStd::string> ROS2PoseControl::GetCurrentTransformViaTF2(
        const AZStd::string& targetFrame,
        const AZStd::string& sourceFrame)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        std::string errorString;
        bool exceptionThrown = false;
        const auto targetFrameCStr = targetFrame.c_str();
        const auto sourceFrameCStr = sourceFrame.c_str();
        if (m_tf_buffer->canTransform(
            targetFrameCStr,
            sourceFrameCStr,
            tf2::TimePointZero,
            &errorString))
        {
            try
            {
                transformStamped = m_tf_buffer->lookupTransform(
                    targetFrameCStr,
                    sourceFrameCStr,
                    tf2::TimePointZero);
                m_tf2WarningShown = false;
            } catch (const tf2::TransformException& ex)
            {
                errorString = ex.what();
                exceptionThrown = true;
            }
        }
        if (exceptionThrown || !errorString.empty())
        {
            if (!m_tf2WarningShown)
            {
                AZ_Warning(
                    "ROS2PositionControl",
                    false,
                    "Could not transform %s to %s, error: %s",
                    targetFrameCStr,
                    sourceFrameCStr,
                    errorString.c_str());
                m_tf2WarningShown = true;
            }
            return AZ::Failure("Could not transform");
        }
        const AZ::Quaternion rotation = ROS2::ROS2Conversions::FromROS2Quaternion(transformStamped.transform.rotation);
        const AZ::Vector3 translation = ROS2::ROS2Conversions::FromROS2Vector3(transformStamped.transform.translation);
        return AZ::Success(AZ::Transform::CreateFromQuaternionAndTranslation(rotation, translation));
    }

    void ROS2PoseControl::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::TF2)
        {
            const AZ::Outcome<AZ::Transform, AZStd::string> transform_outcome = GetCurrentTransformViaTF2(m_configuration.m_referenceFrame,m_configuration.m_targetFrame);
            if (!transform_outcome.IsSuccess())
            {
                return;
            }
            else
            {
                ApplyTransform(transform_outcome.GetValue());
            }
        }
    }

    void ROS2PoseControl::OnTopicConfigurationChanged()
    {
        if (m_configuration.m_tracking_mode != ROS2PoseControlConfiguration::TrackingMode::PoseMessages)
        {
            return;
        }
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        const auto* ros2_frame_component = m_entity->FindComponent<ROS2::ROS2FrameComponent>();
        auto namespaced_topic_name =
            ROS2::ROS2Names::GetNamespacedName(ros2_frame_component->GetNamespace(), m_configuration.m_poseTopicConfiguration.m_topic);
        auto frameId = ros2_frame_component->GetFrameID();
        m_poseSubscription = ros2Node->create_subscription<geometry_msgs::msg::PoseStamped>(
            namespaced_topic_name.data(),
            m_configuration.m_poseTopicConfiguration.GetQoS(),
            [frameId, this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                if (m_configuration.m_tracking_mode != ROS2PoseControlConfiguration::TrackingMode::PoseMessages || !m_isTracking)
                {
                    return;
                }
                const AZ::Transform requestedTransform = ROS2::ROS2Conversions::FromROS2Pose(msg->pose);
                AZ::Transform finalTransform;
                if (msg->header.frame_id.empty())
                {
                    finalTransform = requestedTransform;
                }
                else if (frameId.compare(msg->header.frame_id.c_str()) == 0)
                {
                    auto offsetTransformOptional = GetOffsetTransform(m_configuration.m_startOffsetTag);
                    AZ::Transform offsetTransform = offsetTransformOptional.has_value() ? offsetTransformOptional.value() : AZ::Transform::CreateIdentity();
                    offsetTransform.Invert();

                    auto entityTransform = GetEntity()->GetTransform()->GetWorldTM();

                    finalTransform = offsetTransform * entityTransform * requestedTransform;
                }
                else
                {
                    AZStd::string headerFrameId(msg->header.frame_id.c_str());
                    
                    const AZ::Outcome<AZ::Transform, AZStd::string> transform_outcome = GetCurrentTransformViaTF2(
                        m_configuration.m_referenceFrame,
                        headerFrameId);
                    if (transform_outcome.IsSuccess())
                    {
                        finalTransform = transform_outcome.GetValue() * requestedTransform;
                    }
                    else
                    {
                        AZ_Warning("ROS2PoseControl", true, "No transform found from refrence frame (%s) to requested frame (%s)\n",m_configuration.m_referenceFrame.c_str(), headerFrameId.c_str());
                        return;
                    }
                }

                ApplyTransform(finalTransform);
            });
    }

    void ROS2PoseControl::OnIsTrackingChanged()
    {
        if (m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::TF2)
        {
            if (m_isTracking)
            {
                if (!AZ::TickBus::Handler::BusIsConnected())
                {
                    AZ::TickBus::Handler::BusConnect();
                }
            }
            else
            {
                AZ::TickBus::Handler::BusDisconnect();
            }
        }
    }

    void ROS2PoseControl::SetIsTracking(const bool isTracking)
    {
        m_isTracking = isTracking;
        OnIsTrackingChanged();
    }

    void ROS2PoseControl::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2PoseControl, AZ::Component>()
                ->Version(1)
                ->Field("m_isTracking", &ROS2PoseControl::m_isTracking)
                ->Field("m_configuration", &ROS2PoseControl::m_configuration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2PoseControl>("ROS2PoseControl", "A component that controls the pose of the entity based on ROS 2 data.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "ROS2PoseControl")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2PoseControl::m_isTracking, "Is Tracking", "Turn the tracking on or off")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControl::m_configuration,
                        "Configuration",
                        "Configuration for ROS2PoseControl component");
            }
        }
    }

    void ROS2PoseControl::OnImGuiUpdate()
    {
        std::stringstream ss;
        ss << "ROS2PoseControl Entity: " << GetEntity()->GetName().c_str() << " , Id: " << GetEntityId().ToString().c_str();
        ImGui::Begin(ss.str().c_str());
        if (ImGui::Checkbox("Is Tracking", &m_isTracking))
        {
            OnIsTrackingChanged();
        }
        ImGui::Checkbox("Lock Z Axis", &m_configuration.m_lockZAxis);
        ImGui::Checkbox("Clamp to Ground", &m_configuration.m_clampToGround);
        ImGui::Text(
            "Tracking Mode: %s",
            m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::TF2 ? "TF2" : "Pose Messages");

        ImGui::Text(
            "Position %f %f %f",
            GetEntity()->GetTransform()->GetWorldTranslation().GetX(),
            GetEntity()->GetTransform()->GetWorldTranslation().GetY(),
            GetEntity()->GetTransform()->GetWorldTranslation().GetZ());
        ImGui::Text(
            "Rotation %f %f %f",
            GetEntity()->GetTransform()->GetWorldRotation().GetX(),
            GetEntity()->GetTransform()->GetWorldRotation().GetY(),
            GetEntity()->GetTransform()->GetWorldRotation().GetZ());
        ImGui::End();
    }

    AZ::Transform ROS2PoseControl::RemoveTilt(AZ::Transform transform) const
    {
        const AZ::Vector3 axisX = transform.GetBasisX();
        const AZ::Vector3 axisY = transform.GetBasisY();

        const AZ::Matrix3x3 projectionOnXY{ AZ::Matrix3x3::CreateFromColumns(
            AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateZero()) };

        const AZ::Vector3 newAxisZ = AZ::Vector3::CreateAxisZ(); // new axis Z points up

        // project axisX on the XY plane
        const AZ::Vector3 projectedAxisX = (projectionOnXY * axisX);
        const AZ::Vector3 projectedAxisY = (projectionOnXY * axisY);

        AZ::Vector3 newAxisX = AZ::Vector3::CreateZero();
        AZ::Vector3 newAxisY = AZ::Vector3::CreateZero();

        // get 3rd vector of the new basis from the cross product of the projected vectors.
        // Primarily we want to use the projectedAxisX as the newAxisX, but if it is zero-length, we use the projectedAxisY as the newAxisY.
        if (!projectedAxisX.IsZero())
        {
            newAxisX = projectedAxisX.GetNormalized();
            newAxisY = newAxisZ.Cross(newAxisX);
        }
        else
        {
            newAxisY = projectedAxisY.GetNormalized();
            newAxisX = newAxisY.Cross(newAxisZ);
        }
        // apply rotation using created basis
        transform.SetRotation(AZ::Quaternion::CreateFromBasis(newAxisX, newAxisY, newAxisZ));
        return transform;
    }

    AZStd::optional<AZ::Transform> ROS2PoseControl::GetOffsetTransform(const AZStd::string& tagName)
    {
        AZStd::optional<AZ::Transform> offsetTransform = AZStd::nullopt;
        AZ::EBusAggregateResults<AZ::EntityId> aggregator;
        const LmbrCentral::Tag tag = AZ::Crc32(tagName);
        LmbrCentral::TagGlobalRequestBus::EventResult(aggregator, tag, &LmbrCentral::TagGlobalRequests::RequestTaggedEntities);
        if (!m_startingOffsetNotFoundWarningShown)
        {
            AZ_Warning(
                "ROS2PoseControl",
                aggregator.values.size() <= 1,
                "Multiple entities found with tag %s. The first entity will be used.",
                tagName.c_str());

            AZ_Warning("ROS2PoseControl", !aggregator.values.empty(), "No entity with tag found %s.", tagName.c_str());
            m_startingOffsetNotFoundWarningShown = aggregator.values.size() != 1;
        }
        if (aggregator.values.size() == 1)
        {
            m_startingOffsetNotFoundWarningShown = false;
        }
        if (!aggregator.values.empty())
        {
            const AZ::EntityId& entityId = aggregator.values[0];
            AZ::TransformBus::EventResult(offsetTransform, entityId, &AZ::TransformBus::Events::GetWorldTM);
        }

        return offsetTransform;
    }

    void ROS2PoseControl::ApplyTransform(const AZ::Transform& transform)
    {
        if (!m_isPhysicsDisabled)
        {
            DisablePhysics();
            m_isPhysicsDisabled = true;
        }
        AZ::Transform modifiedTransform = m_configuration.m_lockZAxis ? RemoveTilt(transform) : transform;
        if (!m_configuration.m_startOffsetTag.empty())
        {
            auto startOffsetTransform = GetOffsetTransform(m_configuration.m_startOffsetTag);
            if (startOffsetTransform.has_value())
            {
                modifiedTransform = startOffsetTransform.value() * modifiedTransform;
            }
        }
        if (m_configuration.m_clampToGround)
        {
            constexpr float maxDistance = 40.0f;
            const AZ::Vector3 gravityDirection = -AZ::Vector3::CreateAxisZ();
            const AZ::Vector3 location = modifiedTransform.GetTranslation();
            const AZ::Vector3 locationAbove = location + AZ::Vector3::CreateAxisZ() * maxDistance / 2.0f;
            AZStd::optional<AZ::Vector3> hitPosition = QueryGround(locationAbove, gravityDirection, maxDistance);
            if (hitPosition.has_value())
            {
                modifiedTransform.SetTranslation(hitPosition.value() - gravityDirection * m_configuration.m_groundOffset);
            }
        }
        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, modifiedTransform);
    }

    AZStd::optional<AZ::Vector3> ROS2PoseControl::QueryGround(
        const AZ::Vector3& location, const AZ::Vector3& gravityDirection, float maxDistance)
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(physicsSystem, "Unable to get physics system interface");
        AZ_Assert(sceneInterface, "Unable to get physics scene interface");

        AZStd::optional<AZ::Vector3> hitPosition = AZStd::nullopt;
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        if (sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return hitPosition;
        }

        if (!sceneInterface || !physicsSystem)
        {
            return hitPosition;
        }

        AzPhysics::RayCastRequest request;
        request.m_start = location;
        request.m_direction = gravityDirection;
        request.m_distance = maxDistance;

        AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(sceneHandle, &request);
        if(!m_groundNotFoundWarningShown)
        {
            AZ_Warning("ROS2PoseControl", hitPosition.has_value(), "No ground found for the entity");
            m_groundNotFoundWarningShown = !hitPosition.has_value();
        }
        m_groundNotFoundWarningShown = hitPosition.has_value();
        if (!result.m_hits.empty())
        {
            hitPosition = result.m_hits.front().m_position;
        }
        return hitPosition;
    }

} // namespace ROS2PoseControl
