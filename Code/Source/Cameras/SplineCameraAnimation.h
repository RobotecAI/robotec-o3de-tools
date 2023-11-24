/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root
* of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Color.h>
#include <ImGuiBus.h>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ROS2/Communication/TopicConfiguration.h>

namespace ROS2::Demo
{

    class SplineCameraAnimationGlobalRequests
    {
    public:
        AZ_RTTI(SplineCameraAnimationGlobalRequests, "{ff381f92-e35b-4196-b36f-fd1c9e6cb412}");
        virtual ~SplineCameraAnimationGlobalRequests() = default;

        virtual void ResetAll() = 0;
        virtual void SetSpeed(float speed) = 0;
    };

    class SplineCameraAnimationGlobalBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SplineCameraAnimationGlobalBus = AZ::EBus<SplineCameraAnimationGlobalRequests, SplineCameraAnimationGlobalBusTraits>;

    class SplineCameraAnimation
        : public AZ::Component
        , public AZ::TickBus::Handler
        , protected ImGui::ImGuiUpdateListenerBus::Handler
        , protected SplineCameraAnimationGlobalBus::Handler
    {

    public:
        AZ_COMPONENT(SplineCameraAnimation, "1eb76f57-01f4-41aa-834d-b4138272005a");

        SplineCameraAnimation();

        ~SplineCameraAnimation() = default;

        // AZ::Component overrides...
        void Activate() override;

        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

        // SplineCameraAnimationGlobalBus::Handler overrides...
        void ResetAll() override;
        void SetSpeed(float speed) override;

    private:
        void SetVisibility(bool visible);
        // ImGui::ImGuiUpdateListenerBus::Handler overrides...
        void OnImGuiUpdate() override;

        // AZ::TickBus::Handler overrides...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        float m_currentTrackTime = 0.0f; //!< Time in seconds since the start of the track
        float m_trackLength = 0.0f;
        float m_trackSpeed = 0.1f;
        float m_phaseShift = 0.0f; //!< Phase shift in meters
        float m_normalizedPose = 0.0f;
        AZ::EntityId m_entityToAnimate {AZ::EntityId::InvalidEntityId};
        AZ::Transform m_localTransform = AZ::Transform::CreateIdentity();

        float m_hideTimeOffset = 0.95f;
        AZStd::vector<AZ::EntityId> m_hideEntities;
        bool m_visible {true};

        ROS2::TopicConfiguration m_joystickTopicConfiguration;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> m_joystickSubscription;
    };
} // namespace ROS2::Demo
