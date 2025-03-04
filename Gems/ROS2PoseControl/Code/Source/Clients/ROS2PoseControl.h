/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "ROS2PoseControlConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/containers/vector.h>
#include <ImGuiBus.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2PoseControl/ROS2PoseControlRequestBus.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ROS2PoseControl
{
    class ROS2PoseControl
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , public ROS2PoseControlRequestsBus::Handler

    {
    public:
        AZ_COMPONENT(ROS2PoseControl, "{75e9cdd4-e460-4bea-b9f6-c673f820fb4c}", AZ::Component);

        ROS2PoseControl();

        ~ROS2PoseControl() = default;

        // AZ::Component overrides.
        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

        // AZ::TickBus::Handler overrides.
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    private:
        //! Obtains the transform in the world tagged with the given tag name.
        //! @param tagName The name of the tag to search for.
        //! @return The transform of the entity with the given tag name, if it exists.
        AZStd::optional<AZ::Transform> GetOffsetTransform(const AZStd::string& tagName);

        // ImGui::ImGuiUpdateListenerBus::Handler overrides.
        void OnImGuiUpdate() override;

        //! Obtains the current transform between m_config.m_targetFrame and m_config.m_referenceFrame using TF2.
        [[nodiscard]] AZ::Outcome<AZ::Transform, void> GetCurrentTransformViaTF2();

        //! Obtains the transform between the target frame and the source frame using TF2.
        //! @param targetFrame The target frame.
        //! @param sourceFrame The source frame.
        //! @return The transform between the target frame and the source frame.
        AZ::Outcome<AZ::Transform, void> GetTF2Transform(const AZStd::string& targetFrame, const AZStd::string& sourceFrame);

        //! Obtains the transform from a PoseStamped message. Uses TF2 to find the transform between the global frame and the frame in the
        //! PoseStamped message, then applies it to the transform in the PoseStamped message. If the global frame is not found, the
        //! transform is assumed to be in the global frame.
        //! @param msg The PoseStamped message.
        //! @return The world transform translated using the global frame and the pose stamped.
        [[nodiscard]] AZ::Outcome<AZ::Transform, void> GetTransformFromPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        //! Removes the tilt from a transform by projecting the transform's forward vector onto the gravity direction and creating a new
        //! basis from the projected forward vector and the gravity direction.
        //! @param transform The transform to remove the tilt from.
        //! @return The transform with the tilt removed.
        AZ::Transform RemoveTilt(AZ::Transform transform) const;

        //! Queries the ground at a location using the gravity direction and a maximum distance.
        //! @param location The location to query the ground at.
        //! @param gravityDirection The direction of gravity.
        //! @param maxDistance The maximum distance to query the ground.
        //! @return The ground location if it is found, or an empty optional if the ground is not found.
        AZStd::optional<AZ::Vector3> QueryGround(const AZ::Vector3& location, const AZ::Vector3& gravityDirection, float maxDistance);

        //! Applies the given transform to the entity. Run when a transform is received from a PoseStamped message.
        //! @param transform The transform to apply.
        void OnPoseMessageReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        //! Applies the given transform to the entity. Will check if removing tilt, applying an offset, and clamping to the ground are
        //! enabled in the configuration and add those modifications to the input transform.
        //! @param transform The transform to apply.
        void ApplyTransform(const AZ::Transform& transform);

        //! Enables physics on the entity and its descendants. Only enables physics of entities that had physics disabled by this component.
        void EnablePhysics();
        //! Disables physics on the entity and its descendants.
        void DisablePhysics();
        //! Sets the physics state of the entity and its descendants. Records the entities that had physics disabled by this component and
        //! enables only those entities.
        void SetPhysicsEnabled(bool enabled);

        // ROS2PoseControlRequestsBus::Handler overrides.
        void SetTrackingMode(TrackingMode trackingMode) override;
        void SetTargetFrame(const AZStd::string& targetFrame) override;
        void SetReferenceFrame(const AZStd::string& referenceFrame) override;
        void SetEnablePhysics(bool enable) override;
        void SetRigidBodiesToKinematic(bool enable) override;
        void ApplyConfiguration() override;

        //! Initializes all ROS2-related things (rclcpp::Subscription, tf2_ros::Buffer etc.)
        //! Allows to reconfigure tracking mode in the runtime
        void InitializeROSConnection();
        //! Deinitializes all ROS2-related things (rclcpp::Subscription, tf2_ros::Buffer etc.)
        //! Allows to reconfigure tracking mode in the runtime
        void DeinitializeROSConnection();

        // Tracks the entities that need physics reenabled.
        AZStd::unordered_set<AZ::EntityId> m_needsPhysicsReenable;

        // Configuration
        ROS2PoseControlConfiguration m_configuration;
        bool m_configurationChanged{ false };

        // Pose Messages Tracking.
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> m_poseSubscription;

        // Restoring prefab if m_enablePhysics == false or m_isKinematic == true
        bool m_initialPositionRestored{ false };
        AZStd::unordered_map<AZ::EntityId, AZ::Transform> m_localTransforms;

        // TF2 Tracking.
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{ nullptr };
        std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
        AZStd::string m_odomFrameId;
        bool m_tfWarningLogShown{ false };
    };
} // namespace ROS2PoseControl
