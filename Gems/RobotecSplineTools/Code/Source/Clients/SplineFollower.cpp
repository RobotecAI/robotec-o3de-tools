/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SplineFollower.h"

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Casting/numeric_cast.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Color.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/limits.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>

#include <cmath>
#include <utility>

namespace SplineTools
{
    namespace
    {
        //! Upper bound on samples taken per progress search, so that a wide window or a fine resolution
        //! cannot turn re-localization into a per-frame cost problem.
        constexpr float MaxSearchSamples = 512.0f;
        constexpr float MinSearchResolution = 0.01f;
        constexpr float DebugSphereRadius = 0.15f;
    } // namespace

    SplineFollowerConfiguration::SplineFollowerConfiguration()
    {
        m_topicConfig.m_type = "geometry_msgs::msg::Twist";
        m_topicConfig.m_topic = "cmd_vel";
    }

    void SplineFollowerConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (const auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineFollowerConfiguration>()
                ->Version(0)
                ->Field("m_topicConfig", &SplineFollowerConfiguration::m_topicConfig)
                ->Field("m_splineEntityId", &SplineFollowerConfiguration::m_splineEntityId)
                ->Field("m_poseEntityId", &SplineFollowerConfiguration::m_poseEntityId)
                ->Field("m_startOnActivate", &SplineFollowerConfiguration::m_startOnActivate)
                ->Field("m_publishFrequency", &SplineFollowerConfiguration::m_publishFrequency)
                ->Field("m_maxLinearSpeed", &SplineFollowerConfiguration::m_maxLinearSpeed)
                ->Field("m_maxAngularSpeed", &SplineFollowerConfiguration::m_maxAngularSpeed)
                ->Field("m_lookaheadDistance", &SplineFollowerConfiguration::m_lookaheadDistance)
                ->Field("m_crossTrackGain", &SplineFollowerConfiguration::m_crossTrackGain)
                ->Field("m_headingGain", &SplineFollowerConfiguration::m_headingGain)
                ->Field("m_headingSlowdownAngle", &SplineFollowerConfiguration::m_headingSlowdownAngle)
                ->Field("m_goalTolerance", &SplineFollowerConfiguration::m_goalTolerance)
                ->Field("m_searchWindowBackward", &SplineFollowerConfiguration::m_searchWindowBackward)
                ->Field("m_searchWindowForward", &SplineFollowerConfiguration::m_searchWindowForward)
                ->Field("m_searchResolution", &SplineFollowerConfiguration::m_searchResolution)
                ->Field("m_debugDraw", &SplineFollowerConfiguration::m_debugDraw);

            if (const auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SplineFollowerConfiguration>("SplineFollowerConfiguration", "Configuration for the SplineFollower component")
                    ->ClassElement(AZ::Edit::ClassElements::Group, "SplineFollower Configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_splineEntityId,
                        "Spline Entity",
                        "Entity with the Spline component to follow.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_poseEntityId,
                        "Pose Entity",
                        "Entity whose transform is compared against the spline. Leave unset to use this entity.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_startOnActivate,
                        "Start On Activate",
                        "Begin following as soon as the component activates. If disabled, call StartFollowing on the SplineFollowerRequestBus.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_publishFrequency,
                        "Publish Frequency",
                        "Rate of published Twist messages in Hz. Zero publishes once per tick.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " Hz")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_maxLinearSpeed,
                        "Max Linear Speed",
                        "Forward speed commanded when the entity is aligned with the path.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " m/s")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_maxAngularSpeed,
                        "Max Angular Speed",
                        "Absolute limit applied to the commanded yaw rate.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " rad/s")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_lookaheadDistance,
                        "Lookahead Distance",
                        "How far ahead along the spline the pursuit target sits. Larger is smoother but cuts corners.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_crossTrackGain,
                        "Cross Track Gain",
                        "Steering added per meter of lateral offset. Raise this to hug the spline more tightly.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_headingGain,
                        "Heading Gain",
                        "Steering applied per radian of heading error while turning in place.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_headingSlowdownAngle,
                        "Heading Slowdown Angle",
                        "Heading error at which forward speed reaches zero, so the entity turns before driving.")
                    ->Attribute(AZ::Edit::Attributes::Min, 1.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_goalTolerance,
                        "Goal Tolerance",
                        "Distance from the end of an open spline at which following is considered complete.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_searchWindowBackward,
                        "Search Window Backward",
                        "How far back along the spline progress may be re-localized. Keep small to enforce forward progress.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_searchWindowForward,
                        "Search Window Forward",
                        "How far ahead along the spline progress may be re-localized in one update.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_searchResolution,
                        "Search Resolution",
                        "Spacing of samples used to re-localize progress along the spline.")
                    ->Attribute(AZ::Edit::Attributes::Min, MinSearchResolution)
                    ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineFollowerConfiguration::m_debugDraw,
                        "Debug Draw",
                        "Draw the tracked point and the lookahead target in the viewport.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SplineFollowerConfiguration::m_topicConfig, "Topic Config", "Topic Config");
            }
        }
    }

    SplineFollower::SplineFollower(SplineFollowerConfiguration config)
        : m_config(std::move(config))
    {
    }

    void SplineFollower::Reflect(AZ::ReflectContext* context)
    {
        SplineFollowerConfiguration::Reflect(context);
        if (const auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineFollower, AZ::Component>()->Version(0)->Field("m_config", &SplineFollower::m_config);

            if (const auto editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<SplineFollower>("SplineFollower", "Follows a spline by publishing Twist velocity commands to cmd_vel.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "SplineFollower")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SplineFollower::m_config, "Configuration", "Configuration for the SplineFollower component");
            }
        }
    }

    void SplineFollower::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void SplineFollower::Activate()
    {
        AZStd::string ros2Namespace;
        ROS2::ROS2FrameComponentBus::EventResult(
            ros2Namespace, GetEntityId(), &ROS2::ROS2FrameComponentRequests::GetNamespace);

        // The configured topic is kept namespace-free so that repeated activations cannot stack
        // namespace prefixes onto the serialized value.
        AZStd::string topicName = m_config.m_topicConfig.m_topic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            topicName, &ROS2::ROS2NamesRequests::GetNamespacedName, ros2Namespace, m_config.m_topicConfig.m_topic);

        const auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (!ros2Node)
        {
            AZ_Error("SplineFollower", false, "ROS 2 node is not available, cannot publish velocity commands.");
            return;
        }
        m_publisher = ros2Node->create_publisher<geometry_msgs::msg::Twist>(topicName.c_str(), m_config.m_topicConfig.GetQoS());

        if (m_config.m_debugDraw)
        {
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(AZ::RPI::Scene::GetSceneForEntityId(GetEntityId()));
        }

        SplineFollowerRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();

        if (m_config.m_startOnActivate)
        {
            StartFollowing();
        }
    }

    void SplineFollower::Deactivate()
    {
        // Leave the robot with a stop command rather than the last velocity it was given.
        if (m_following)
        {
            PublishTwist(0.0f, 0.0f);
            m_following = false;
        }

        AZ::TickBus::Handler::BusDisconnect();
        SplineFollowerRequestBus::Handler::BusDisconnect();
        m_drawQueue = nullptr;
        m_publisher.reset();
    }

    void SplineFollower::StartFollowing()
    {
        if (!m_config.m_splineEntityId.IsValid())
        {
            AZ_Error("SplineFollower", false, "No spline entity is configured, cannot start following.");
            return;
        }

        m_following = true;
        // Force a full-spline search on the next update so that following resumes from wherever the
        // entity actually is, not from stale progress.
        m_progressValid = false;
        m_timeSinceLastPublish = 0.0f;
    }

    void SplineFollower::StopFollowing()
    {
        if (!m_following)
        {
            return;
        }
        m_following = false;
        PublishTwist(0.0f, 0.0f);
    }

    bool SplineFollower::IsFollowing() const
    {
        return m_following;
    }

    float SplineFollower::GetDistanceAlongSpline() const
    {
        return m_distanceAlongSpline;
    }

    float SplineFollower::GetCrossTrackError() const
    {
        return m_crossTrackError;
    }

    void SplineFollower::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_following)
        {
            return;
        }

        m_timeSinceLastPublish += deltaTime;
        if (m_config.m_publishFrequency > 0.0f && m_timeSinceLastPublish < 1.0f / m_config.m_publishFrequency)
        {
            return;
        }
        m_timeSinceLastPublish = 0.0f;

        switch (Update())
        {
        case UpdateResult::Continue:
            break;
        case UpdateResult::Finished:
            StopFollowing();
            SplineFollowerNotificationBus::Event(GetEntityId(), &SplineFollowerNotifications::OnSplineFollowingFinished);
            break;
        case UpdateResult::Error:
            StopFollowing();
            break;
        }
    }

    SplineFollower::UpdateResult SplineFollower::Update()
    {
        AZ::SplinePtr spline;
        LmbrCentral::SplineComponentRequestBus::EventResult(
            spline, m_config.m_splineEntityId, &LmbrCentral::SplineComponentRequests::GetSpline);
        if (!spline)
        {
            AZ_Error("SplineFollower", false, "Entity %s has no Spline component.", m_config.m_splineEntityId.ToString().c_str());
            return UpdateResult::Error;
        }

        const float splineLength = spline->GetSplineLength();
        if (splineLength <= AZ::Constants::FloatEpsilon)
        {
            AZ_Error("SplineFollower", false, "Spline has zero length; it needs at least two distinct vertices.");
            return UpdateResult::Error;
        }

        AZ::Transform splineTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(splineTransform, m_config.m_splineEntityId, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Transform entityTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(entityTransform, GetPoseEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        const AZ::Transform worldToSpline = splineTransform.GetInverse();
        const AZ::Transform worldToEntity = entityTransform.GetInverse();

        // Spline queries operate in the spline entity's local space.
        const AZ::Vector3 positionInSpline = worldToSpline.TransformPoint(entityTransform.GetTranslation());

        const bool closed = spline->IsClosed();
        if (m_progressValid)
        {
            m_distanceAlongSpline = FindProgressInWindow(*spline, positionInSpline, splineLength);
        }
        else
        {
            const AZ::PositionSplineQueryResult nearest = spline->GetNearestAddressPosition(positionInSpline);
            m_distanceAlongSpline = spline->GetLength(nearest.m_splineAddress);
            m_progressValid = true;
        }

        const AZ::Vector3 closestPoint =
            splineTransform.TransformPoint(spline->GetPosition(spline->GetAddressByDistance(m_distanceAlongSpline)));

        // Expressed in the entity frame, the lateral component of the nearest path point is the signed
        // cross-track error: positive means the path lies to the entity's left.
        m_crossTrackError = worldToEntity.TransformPoint(closestPoint).GetY();

        if (!closed && splineLength - m_distanceAlongSpline <= m_config.m_goalTolerance)
        {
            return UpdateResult::Finished;
        }

        const float lookaheadProgress = NormalizeDistance(m_distanceAlongSpline + m_config.m_lookaheadDistance, splineLength, closed);
        const AZ::Vector3 lookaheadPoint =
            splineTransform.TransformPoint(spline->GetPosition(spline->GetAddressByDistance(lookaheadProgress)));
        const AZ::Vector3 lookaheadInEntity = worldToEntity.TransformPoint(lookaheadPoint);

        // Steering is planar: a differential-drive command has no way to express the vertical component,
        // so height differences along the spline must not leak into the heading.
        const float forward = lookaheadInEntity.GetX();
        const float lateral = lookaheadInEntity.GetY();
        const float headingError = std::atan2(lateral, forward);

        // Throttle forward speed as heading error grows, so a badly misaligned entity turns towards the
        // path rather than driving away from it.
        const float slowdownAngle = AZ::DegToRad(AZ::GetMax(m_config.m_headingSlowdownAngle, 1.0f));
        const float alignment = AZ::GetClamp(1.0f - std::abs(headingError) / slowdownAngle, 0.0f, 1.0f);
        const float linearVelocity = m_config.m_maxLinearSpeed * alignment;

        // Pure pursuit: curvature of the circular arc from the entity to the lookahead point.
        const float lookaheadRangeSq = forward * forward + lateral * lateral;
        const float pursuitTerm =
            lookaheadRangeSq > AZ::Constants::FloatEpsilon ? linearVelocity * 2.0f * lateral / lookaheadRangeSq : 0.0f;

        // The pursuit term is proportional to linear speed, so it vanishes exactly when the entity is
        // throttled down. This term takes over there and rotates it back towards the path.
        const float headingTerm = m_config.m_headingGain * headingError * (1.0f - alignment);

        // Pure pursuit only aims at the lookahead point and will happily settle on a track parallel to
        // the spline. This term is what removes the standing offset.
        const float crossTrackTerm = m_config.m_crossTrackGain * m_crossTrackError;

        const float angularVelocity =
            AZ::GetClamp(pursuitTerm + headingTerm + crossTrackTerm, -m_config.m_maxAngularSpeed, m_config.m_maxAngularSpeed);

        PublishTwist(linearVelocity, angularVelocity);
        DrawDebug(closestPoint, lookaheadPoint);

        return UpdateResult::Continue;
    }

    float SplineFollower::NormalizeDistance(float distance, float splineLength, bool closed) const
    {
        if (!closed)
        {
            return AZ::GetClamp(distance, 0.0f, splineLength);
        }

        float wrapped = std::fmod(distance, splineLength);
        if (wrapped < 0.0f)
        {
            wrapped += splineLength;
        }
        return wrapped;
    }

    float SplineFollower::FindProgressInWindow(const AZ::Spline& spline, const AZ::Vector3& localPosition, float splineLength) const
    {
        const bool closed = spline.IsClosed();
        const float backward = AZ::GetMax(m_config.m_searchWindowBackward, 0.0f);
        const float forward = AZ::GetMax(m_config.m_searchWindowForward, 0.0f);

        // Widen the step rather than the budget when the window is large, so the cost per update stays bounded.
        const float resolution =
            AZ::GetMax(AZ::GetMax(m_config.m_searchResolution, (backward + forward) / MaxSearchSamples), MinSearchResolution);

        float bestProgress = m_distanceAlongSpline;
        float bestDistanceSq = AZStd::numeric_limits<float>::max();
        for (float offset = -backward; offset <= forward; offset += resolution)
        {
            const float candidate = NormalizeDistance(m_distanceAlongSpline + offset, splineLength, closed);
            const AZ::Vector3 point = spline.GetPosition(spline.GetAddressByDistance(candidate));
            const float distanceSq = (point - localPosition).GetLengthSq();
            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
                bestProgress = candidate;
            }
        }
        return bestProgress;
    }

    AZ::EntityId SplineFollower::GetPoseEntityId() const
    {
        return m_config.m_poseEntityId.IsValid() ? m_config.m_poseEntityId : GetEntityId();
    }

    void SplineFollower::PublishTwist(float linearVelocity, float angularVelocity) const
    {
        if (!m_publisher)
        {
            return;
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x = linearVelocity;
        twist.angular.z = angularVelocity;
        m_publisher->publish(twist);
    }

    void SplineFollower::DrawDebug(const AZ::Vector3& closestPoint, const AZ::Vector3& lookaheadPoint) const
    {
        if (!m_config.m_debugDraw || !m_drawQueue)
        {
            return;
        }

        AZ::Transform entityTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(entityTransform, GetPoseEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        m_drawQueue->DrawSphere(closestPoint, DebugSphereRadius, AZ::Colors::Green);
        m_drawQueue->DrawSphere(lookaheadPoint, DebugSphereRadius, AZ::Colors::Yellow);

        const AZStd::array<AZ::Vector3, 2> pursuitLine = { entityTransform.GetTranslation(), lookaheadPoint };
        AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
        drawArgs.m_verts = pursuitLine.data();
        drawArgs.m_vertCount = aznumeric_cast<uint32_t>(pursuitLine.size());
        drawArgs.m_colors = &AZ::Colors::Yellow;
        drawArgs.m_colorCount = 1u;
        drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
        drawArgs.m_size = 1u;
        m_drawQueue->DrawLines(drawArgs);
    }
} // namespace SplineTools
