/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "SplineCameraAnimation.h"
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <imgui/imgui.h>

namespace ROS2::Demo
{
    SplineCameraAnimation::SplineCameraAnimation()
    {
    }

    void SplineCameraAnimation::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SplineCameraAnimation>()
                ->Version(1)
                ->Field("trackSpeed", &SplineCameraAnimation::m_trackSpeed)
                ->Field("entityToAnimate", &SplineCameraAnimation::m_entityToAnimate)
                ->Field("localRotatio", &SplineCameraAnimation::m_localTransform)
                ->Field("phaseShift", &SplineCameraAnimation::m_phaseShift)
                ->Field("hideTimeOffset", &SplineCameraAnimation::m_hideTimeOffset)
                ->Field("hideEntities", &SplineCameraAnimation::m_hideEntities)
                ->Field("joystickTopicConfiguration", &SplineCameraAnimation::m_joystickTopicConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SplineCameraAnimation>("SplineCameraAnimation", "SplineCameraAnimation.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "CameraJoystick")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "DemoTools")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineCameraAnimation::m_trackSpeed, "trackSpeed", "trackSpeed")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SplineCameraAnimation::m_entityToAnimate, "entityToAnimate", "entityToAnimate")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineCameraAnimation::m_localTransform, "localRotation", "localRotation")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineCameraAnimation::m_phaseShift, "phaseShift", "phaseShift")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SplineCameraAnimation::m_hideTimeOffset, "hideTimeOffset", "hideTimeOffset")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineCameraAnimation::m_hideEntities, "hideEntities", "hideEntities")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineCameraAnimation::m_joystickTopicConfiguration, "topic", "topic");
            }
        }
    }

    void SplineCameraAnimation::Activate()
    {
        m_currentTrackTime = 0.0f;
        SplineCameraAnimationGlobalBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        auto* ros2Interface = ROS2Interface::Get();
        AZ_Assert(ros2Interface, "ROS2 interface not available");
        m_joystickSubscription = ros2Interface->GetNode()->create_subscription<sensor_msgs::msg::Joy>(
            m_joystickTopicConfiguration.m_topic.c_str(),
            m_joystickTopicConfiguration.GetQoS(),
            [&](sensor_msgs::msg::Joy msg)
            {
                m_trackSpeed = 0.010f*(1.0f+msg.axes[3])/2.0f;
                SplineCameraAnimationGlobalBus::Broadcast(&SplineCameraAnimationGlobalBus::Events::SetSpeed, m_trackSpeed);
            });

        m_visible = true;
        SetVisibility(true);
    }

    void SplineCameraAnimation::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        SplineCameraAnimationGlobalBus::Handler::BusDisconnect();
    }

    void SplineCameraAnimation::OnImGuiUpdate()
    {
        AZStd::string label = AZStd::string::format("SplineCameraAnimation%s", this->m_entity->GetName().c_str());
        ImGui::Begin(label.c_str());
        if (ImGui::Button("ResetAll"))
        {
            SplineCameraAnimationGlobalBus::Broadcast(&SplineCameraAnimationGlobalBus::Events::ResetAll);
        }
        float speed = m_trackSpeed;
        ImGui::SliderFloat("Speed", &speed, 0.0001f, 0.05f);

        ImGui::Text("Progress %f ", m_currentTrackTime);
        SplineCameraAnimationGlobalBus::Broadcast(&SplineCameraAnimationGlobalBus::Events::SetSpeed, speed);
        ImGui::End();
    }

    void SplineCameraAnimation::SetSpeed(float speed)
    {
        m_trackSpeed = speed;
    }

    void SplineCameraAnimation::ResetAll()
    {
        AZ_Printf("SplineCameraAnimation", "ResetAll , from %s", this->m_entity->GetName().c_str());
        m_currentTrackTime = 0.0f;
        SetVisibility(true);
        m_visible = true;
    }

    void SplineCameraAnimation::SetVisibility(bool visible)
    {

        for (auto& entityId : m_hideEntities)
        {
            AZStd::vector<AZ::EntityId> descendantIds;
            AZ::TransformBus::EventResult(descendantIds, entityId, &AZ::TransformBus::Events::GetEntityAndAllDescendants);
            for (auto& descendantId : descendantIds)
            {
                AZ::Render::MeshComponentRequestBus::Event(descendantId, &AZ::Render::MeshComponentRequests::SetVisibility, visible);
            }
        }
    }
    void SplineCameraAnimation::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_currentTrackTime > m_hideTimeOffset)
        {
            if (m_visible)
            {
                AZ_Printf("SplineCameraAnimation", "Hide %f / %f", m_currentTrackTime, m_hideTimeOffset);
                SetVisibility(false);
                m_visible = false;
            }

        }

        AZ::Transform splineTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);

        m_currentTrackTime += m_trackSpeed * deltaTime;


        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        float normalizedTrackPosition = m_currentTrackTime + m_phaseShift;
        if (splinePtr->IsClosed())
        {
            m_normalizedPose = AZStd::fmod(normalizedTrackPosition, 1.0f);
        }
        else
        {
            m_normalizedPose = AZStd::min(normalizedTrackPosition, 1.0f);
        }

        const AZ::SplineAddress address = splinePtr->GetAddressByFraction(m_normalizedPose);

        const AZ::Vector3 position = splinePtr->GetPosition(address);
        const AZ::Vector3 tangent = splinePtr->GetTangent(address);
        const AZ::Vector3 normal = splinePtr->GetNormal(address);

        //
        // construct ideal pose as SE(3) - in spline space
        const AZ::Matrix3x3 rot = AZ::Matrix3x3::CreateFromColumns(tangent, normal, tangent.Cross(normal));
        const AZ::Transform goalTransform = AZ::Transform::CreateFromMatrix3x3AndTranslation(rot, position);
        const auto transformOnSpline = splineTransform * goalTransform * m_localTransform;

        AZ::TransformBus::Event(m_entityToAnimate, &AZ::TransformBus::Events::SetWorldTM, transformOnSpline);
    }
} // namespace ROS2::Demo
