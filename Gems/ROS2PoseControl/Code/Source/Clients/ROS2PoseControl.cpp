/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include <ROS2PoseControl/ROS2PoseControl.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Entity/GameEntityContextComponent.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Georeference/GeoreferenceBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2PoseControl/ROS2PoseControlConfiguration.h>
#include <RigidBodyComponent.h>
#include <imgui/imgui.h>
#include <tf2_ros/transform_listener.h>

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
        InitializeRosIntestines();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        ROS2PoseControlRequestsBus::Handler::BusConnect(GetEntityId());

        AZ::TickBus::QueueFunction(
            [this]()
            {
                AZStd::vector<AZ::EntityId> entityIds;
                AZ::TransformBus::EventResult(entityIds, GetEntityId(), &AZ::TransformBus::Events::GetAllDescendants);
                for (const auto& entityId : entityIds)
                {
                    AZ::Transform localTM = AZ::Transform::CreateIdentity();
                    AZ::TransformBus::EventResult(localTM, entityId, &AZ::TransformBus::Events::GetLocalTM);
                    m_localTransforms[entityId] = localTM;
                }
            });
    }

    void ROS2PoseControl::Deactivate()
    {
        DeinitializeRosIntestines();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        ROS2PoseControlRequestsBus::Handler::BusDisconnect();
    }

    void ROS2PoseControl::InitializeRosIntestines()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(ros2Node->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

        if (m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::TF2)
        {
            AZ::TickBus::Handler::BusConnect();
        }
        else if (m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::PoseMessages)
        {
            // Get odom frame Id
            const auto* ros2_frame_component = m_entity->FindComponent<ROS2::ROS2FrameComponent>();
            if (!ros2_frame_component)
            {
                AZ_Error("ROS2PoseControl", false, "ROS2PoseControl requires a ROS2FrameComponent to be present on the entity.");
                return;
            }
            m_odomFrameId = ros2_frame_component->GetGlobalFrameName();

            // Initialize the pose subscription
            auto namespaced_topic_name =
                ROS2::ROS2Names::GetNamespacedName(ros2_frame_component->GetNamespace(), m_configuration.m_poseTopicConfiguration.m_topic);

            m_poseSubscription = ros2Node->create_subscription<geometry_msgs::msg::PoseStamped>(
                namespaced_topic_name.data(),
                m_configuration.m_poseTopicConfiguration.GetQoS(),
                [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    OnPoseMessageReceived(msg);
                }

            );
        }
    }

    void ROS2PoseControl::DeinitializeRosIntestines()
    {
        m_poseSubscription.reset();
        m_tf_buffer.reset();
        m_tf_listener.reset();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ROS2PoseControl::EnablePhysics()
    {
        SetPhysicsEnabled(true);
    }

    void ROS2PoseControl::DisablePhysics()
    {
        SetPhysicsEnabled(false);
    }

    void ROS2PoseControl::SetTrackingMode(const ROS2PoseControlConfiguration::TrackingMode trackingMode)
    {
        m_configuration.m_tracking_mode = trackingMode;
    }

    void ROS2PoseControl::SetTargetFrame(const AZStd::string& targetFrame)
    {
        m_configuration.m_targetFrame = targetFrame;
    }

    void ROS2PoseControl::SetReferenceFrame(const AZStd::string& referenceFrame)
    {
        m_configuration.m_referenceFrame = referenceFrame;
    }

    void ROS2PoseControl::SetEnablePhysics(bool enable)
    {
        SetPhysicsEnabled(enable);
        m_configuration.m_enablePhysics = enable;
    }

    void ROS2PoseControl::SetRigidBodiesToKinematic(bool enable)
    {
        AZStd::vector<AZ::EntityId> entityIds;
        AZ::TransformBus::EventResult(entityIds, GetEntityId(), &AZ::TransformBus::Events::GetAllDescendants);
        entityIds.push_back(GetEntityId());
        for (const AZ::EntityId& entityId : entityIds)
        {
            AZ::Entity* entity;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            if (auto* rigidBodyComponent = entity->FindComponent<PhysX::RigidBodyComponent>())
            {
                rigidBodyComponent->SetKinematic(enable);
            }
        }
        m_configuration.m_isKinematic = enable;
    }

    void ROS2PoseControl::ApplyConfiguration()
    {
        DeinitializeRosIntestines();
        InitializeRosIntestines();
    }

    void ROS2PoseControl::SetPhysicsEnabled(bool enabled)
    {
        AZStd::vector<AZ::EntityId> entityIds;
        AZ::TransformBus::EventResult(entityIds, GetEntityId(), &AZ::TransformBus::Events::GetAllDescendants);
        entityIds.push_back(GetEntityId());
        for (const AZ::EntityId& entityId : entityIds)
        {
            AZ::Entity* entity;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            if (auto* rigidBodyComponent = entity->FindComponent<PhysX::RigidBodyComponent>())
            {
                if (enabled)
                {
                    if (m_needsPhysicsReenable.contains(entityId))
                    {
                        rigidBodyComponent->EnablePhysics();
                        m_needsPhysicsReenable.erase(entityId);
                    }
                }
                else
                {
                    if (rigidBodyComponent->IsPhysicsEnabled())
                    {
                        m_needsPhysicsReenable.insert(entityId);
                        rigidBodyComponent->DisablePhysics();
                    }
                }
            }
        }
    }

    AZ::Outcome<AZ::Transform, void> ROS2PoseControl::GetCurrentTransformViaTF2()
    {
        return GetTF2Transform(m_configuration.m_referenceFrame, m_configuration.m_targetFrame);
    }

    AZ::Outcome<AZ::Transform, void> ROS2PoseControl::GetTF2Transform(const AZStd::string& targetFrame, const AZStd::string& sourceFrame)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        std::string errorString;
        // Check if the transform can be completed. If so return it.
        if (m_tf_buffer->canTransform(targetFrame.c_str(), sourceFrame.c_str(), tf2::TimePointZero, &errorString))
        {
            transformStamped = m_tf_buffer->lookupTransform(targetFrame.c_str(), sourceFrame.c_str(), tf2::TimePointZero);
        }
        else
        {
            AZ_Warning(
                "ROS2PositionControl",
                false,
                "Could not transform %s to %s, error: %s",
                m_configuration.m_targetFrame.c_str(),
                m_configuration.m_referenceFrame.c_str(),
                errorString.c_str());
            return AZ::Failure();
        }

        const AZ::Quaternion rotation = ROS2::ROS2Conversions::FromROS2Quaternion(transformStamped.transform.rotation);
        const AZ::Vector3 translation = ROS2::ROS2Conversions::FromROS2Vector3(transformStamped.transform.translation);
        return AZ::Success(AZ::Transform::CreateFromQuaternionAndTranslation(rotation, translation));
    }

    AZ::Outcome<AZ::Transform, void> ROS2PoseControl::GetTransformFromPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!msg)
        {
            return AZ::Failure();
        }

        // Get the transform from the PoseStamped message
        AZ::Transform transform = ROS2::ROS2Conversions::FromROS2Pose(msg->pose);
        if (msg->header.frame_id.empty())
        {
            return AZ::Success(transform);
        }

        // Get the transform from the global frame to the frame in the PoseStamped message
        const auto globalFrameTransform = GetTF2Transform(m_odomFrameId, msg->header.frame_id.c_str());
        if (!globalFrameTransform.IsSuccess())
        {
            return AZ::Failure();
        }

        // Apply the global frame transform to the transform from the PoseStamped message
        transform = globalFrameTransform.GetValue() * transform;

        return AZ::Success(transform);
    }

    void ROS2PoseControl::OnPoseMessageReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (m_configuration.m_tracking_mode != ROS2PoseControlConfiguration::TrackingMode::PoseMessages)
        {
            return;
        }
        bool isWGS = m_configuration.m_useWGS && msg->header.frame_id == "wgs84";

        if (isWGS && !ROS2::GeoreferenceRequestsBus::HasHandlers())
        {
            AZ_Error(
                "ROS2PoseControl",
                false,
                "Level is not geographically positioned. Cannot convert WGS84 coordinates. Object will not be moved.");
            return;
        }

        AZ::Transform targetTransform;

        if (isWGS)
        {
            ROS2::WGS::WGS84Coordinate coordinate;
            AZ::Vector3 coordinateInLevel = AZ::Vector3(-1);
            AZ::Quaternion rotationInENU = AZ::Quaternion::CreateIdentity();
            coordinate.m_latitude = msg->pose.position.x;
            coordinate.m_longitude = msg->pose.position.y;
            coordinate.m_altitude = msg->pose.position.z;
            ROS2::GeoreferenceRequestsBus::BroadcastResult(rotationInENU, &ROS2::GeoreferenceRequests::GetRotationFromLevelToENU);
            ROS2::GeoreferenceRequestsBus::BroadcastResult(
                coordinateInLevel, &ROS2::GeoreferenceRequests::ConvertFromWGS84ToLevel, coordinate);

            rotationInENU =
                (rotationInENU.GetInverseFast() *
                 AZ::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w))
                    .GetNormalized();

            targetTransform = { coordinateInLevel, rotationInENU, 1.0f };
        }
        else
        {
            auto targetTransformOutcome = GetTransformFromPoseStamped(msg);
            if (!targetTransformOutcome.IsSuccess())
            {
                return;
            }
            targetTransform = targetTransformOutcome.GetValue();
        }
        ApplyTransform(targetTransform);
    }

    void ROS2PoseControl::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_configuration.m_tracking_mode == ROS2PoseControlConfiguration::TrackingMode::TF2)
        {
            const AZ::Outcome<AZ::Transform, void> transform_outcome = GetCurrentTransformViaTF2();
            if (!transform_outcome.IsSuccess())
            {
                return;
            }
            ApplyTransform(transform_outcome.GetValue());
        }
    }

    void ROS2PoseControl::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2PoseControl, AZ::Component>()->Version(1)->Field("m_configuration", &ROS2PoseControl::m_configuration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2PoseControl>("ROS2PoseControl", "A component that controls the pose of the entity based on ROS 2 data.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "ROS2PoseControl")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
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

        AZ_Warning(
            "ROS2PoseControl",
            aggregator.values.size() <= 1,
            "Multiple entities found with tag \"%s\". The first entity will be used.",
            tagName.c_str());

        if (!aggregator.values.empty())
        {
            const AZ::EntityId& entityId = aggregator.values[0];
            AZ::TransformBus::EventResult(offsetTransform, entityId, &AZ::TransformBus::Events::GetWorldTM);
        }
        AZ_Warning(
            "ROS2PoseControl",
            !aggregator.values.empty(),
            "No entity with tag \"%s\" was found. No offset will be applied.",
            tagName.c_str());

        return offsetTransform;
    }

    void ROS2PoseControl::ApplyTransform(const AZ::Transform& transform)
    {
        if (transform.GetRotation().IsZero())
        {
            AZ_Error("ROS2PoseControl", false, "Received invalid rotation. Object will not be moved.");
            return;
        }

        if (m_configuration.m_enablePhysics)
        {
            // Disable physics to allow transform movement.
            DisablePhysics();
        }

        // If prefab has physics disabled or all of its rigid bodies are set to 'Kinematic'
        // then we want to restore the initial positions of all entities. These positions
        // may have changed before the physics or kinematics were enabled/disabled via
        // the EBus request
        if (!m_initialPositionRestored && (!m_configuration.m_enablePhysics || m_configuration.m_isKinematic))
        {
            for (const auto& [entityId, localTM] : m_localTransforms)
            {
                AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, localTM);
            }

            m_initialPositionRestored = true;
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

        if (m_configuration.m_enablePhysics)
        {
            // Re-enable physics after the transform is applied.
            EnablePhysics();
        }
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

        if (!result.m_hits.empty())
        {
            hitPosition = result.m_hits.front().m_position;
        }

        return hitPosition;
    }

} // namespace ROS2PoseControl
