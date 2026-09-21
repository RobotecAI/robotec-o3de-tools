/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Spline.h>
#include <AzCore/Math/Transform.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <SplineTools/SplineFollowerBus.h>
#include <SplineTools/SplineToolsTypeIds.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>

namespace SplineTools
{
    struct SplineFollowerConfiguration
    {
        AZ_TYPE_INFO(SplineFollowerConfiguration, SplineFollowerConfigTypeId);
        static void Reflect(AZ::ReflectContext* context);

        SplineFollowerConfiguration();

        ROS2::TopicConfiguration m_topicConfig{ rclcpp::ServicesQoS() };

        //! Entity holding the Spline component to follow. It is not this entity, which carries the robot.
        AZ::EntityId m_splineEntityId;

        //! Entity whose world transform is taken as the current pose, i.e. the point steered onto the
        //! spline. Leave unset to use this component's own entity. Set it when the component does not
        //! live on the link that should track the path, or to steer from a point offset from the body
        //! such as a front axle.
        AZ::EntityId m_poseEntityId;

        bool m_startOnActivate = true;

        //! Rate at which Twist messages are published, in Hz. Kept independent of the tick rate so that
        //! the command rate stays predictable regardless of frame rate.
        float m_publishFrequency = 20.0f;

        float m_maxLinearSpeed = 0.5f; //!< m/s
        float m_maxAngularSpeed = 0.5f; //!< rad/s

        //! Distance ahead along the spline at which the pursuit target is placed. Larger values give
        //! smoother steering but cut corners; smaller values track tightly but can oscillate.
        float m_lookaheadDistance = 1.0f; //!< m

        //! Steering added per meter of lateral offset from the spline. This is what pulls the entity
        //! back onto the path after the lookahead has let it drift wide.
        float m_crossTrackGain = 1.0f; //!< rad/s per m

        //! Steering applied per radian of heading error. Dominates when the entity is turning in place.
        float m_headingGain = 1.5f; //!< rad/s per rad

        //! Heading error at which linear speed is throttled to zero, so the entity rotates towards the
        //! path instead of driving away from it.
        float m_headingSlowdownAngle = 60.0f; //!< deg

        //! Distance from the spline end at which an open spline is considered complete.
        float m_goalTolerance = 0.3f; //!< m

        //! Progress is re-localized each update by searching only this far back and forward along the
        //! spline. A bounded window keeps the entity from snapping onto a different lobe of a spline
        //! that loops or passes close to itself.
        float m_searchWindowBackward = 1.0f; //!< m
        float m_searchWindowForward = 5.0f; //!< m
        float m_searchResolution = 0.1f; //!< m

        bool m_debugDraw = false;
    };

    //! Drives an entity along a spline by publishing geometry_msgs::msg::Twist to cmd_vel.
    //! The component reads the entity's world transform as the current pose and steers with pure
    //! pursuit plus an explicit cross-track term; it never writes the transform itself, so the robot's
    //! own controller remains the only thing moving it.
    //! Entity +X is taken as forward and +Z as up, matching the ROS 2 body frame convention.
    //! Distances below are spline-local, so they are meters only while the spline entity is unscaled.
    class SplineFollower final
        : public AZ::Component
        , protected AZ::TickBus::Handler
        , protected SplineFollowerRequestBus::Handler
    {
    public:
        AZ_COMPONENT(SplineFollower, SplineFollowerComponentTypeId, AZ::Component);

        SplineFollower() = default;
        explicit SplineFollower(SplineFollowerConfiguration config);
        ~SplineFollower() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // SplineFollowerRequestBus::Handler overrides ...
        void StartFollowing() override;
        void StopFollowing() override;
        bool IsFollowing() const override;
        float GetDistanceAlongSpline() const override;
        float GetCrossTrackError() const override;

    protected:
        // AZ::TickBus::Handler overrides ...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        enum class UpdateResult
        {
            Continue, //!< A velocity command was published; following carries on.
            Finished, //!< The end of an open spline was reached.
            Error //!< The spline could not be read; following cannot continue.
        };

        //! Computes and publishes one velocity command.
        UpdateResult Update();

        //! Maps a distance onto the valid range of the spline: wrapped for closed splines, clamped for open ones.
        float NormalizeDistance(float distance, float splineLength, bool closed) const;

        //! Finds the point on the spline nearest to @p localPosition, searching only within the configured
        //! window around the current progress.
        float FindProgressInWindow(const AZ::Spline& spline, const AZ::Vector3& localPosition, float splineLength) const;

        //! @return The configured pose entity, falling back to this component's entity when unset.
        AZ::EntityId GetPoseEntityId() const;

        void PublishTwist(float linearVelocity, float angularVelocity) const;
        void DrawDebug(const AZ::Vector3& closestPoint, const AZ::Vector3& lookaheadPoint) const;

        SplineFollowerConfiguration m_config;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        bool m_following = false;
        //! Set when following starts, so the first update localizes against the whole spline rather
        //! than a window around stale progress.
        bool m_progressValid = false;
        float m_distanceAlongSpline = 0.0f;
        float m_crossTrackError = 0.0f;
        float m_timeSinceLastPublish = 0.0f;
    };
} // namespace SplineTools
