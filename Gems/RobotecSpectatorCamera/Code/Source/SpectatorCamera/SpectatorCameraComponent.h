#pragma once

#include "SpectatorCameraConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Transform.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>
#include <RobotecSpectatorCamera/RobotecSpectatorCameraBus.h>

namespace RobotecSpectatorCamera
{
    class SpectatorCameraComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AzFramework::InputChannelEventListener
        , public RobotecSpectatorCameraRequestBus::Handler
    {
    public:
        AZ_COMPONENT(SpectatorCameraComponent, SpectatorCameraComponentTypeId);

        SpectatorCameraComponent() = default;
        SpectatorCameraComponent(const SpectatorCameraConfiguration& spectatorCameraConfiguration);
        ~SpectatorCameraComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AzFramework::InputChannelEventListener overrides
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

        // RobotecSpectatorCamera::RobotecSpectatorCameraRequestBus::Handler overrides
        CameraMode GetCameraMode() const override;
        void SetCameraMode(const CameraMode cameraMode) override;
        float GetMouseSensitivity() const override;
        void SetMouseSensitivity(const float mouseSensitivity) override;
        float GetCameraSpeed() const override;
        void SetCameraSpeed(const float cameraSpeed) override;
        bool GetFollowTargetRotation() const override;
        void SetFollowTargetRotation(const bool followTargetRotation) override;
        float GetVerticalOffset() const override;
        void SetVerticalOffset(const float verticalOffset) override;

    private:
        void MouseEvent(const AzFramework::InputChannel& inputChannel);
        void KeyboardEvent(const AzFramework::InputChannel& inputChannel);

        AZ::Vector2 GetCurrentMousePosition() const;
        void RotateCameraOnMouse(const AZ::Vector2& mouseDelta);
        void ToggleCameraMode();

        static constexpr float scrollValueDivider = 1200.0f;
        static constexpr float pitchDegLimit = 87.0f;

        SpectatorCameraConfiguration m_configuration;
        float m_orbitRadius{ 10.0f };
        float m_pitch{ 0.5f };
        float m_yaw{ 0.0f };
        bool m_isRightMouseButtonPressed{ false };
        bool m_ignoreNextMovement{ false };
        AZ::Vector2 m_initialMousePosition;
        AZ::Vector2 m_lastMousePosition{ AZ::Vector2::CreateZero() };
        AZ::Vector3 m_movement{ AZ::Vector3::CreateZero() };
        AZ::Vector2 m_rotation{ AZ::Vector2::CreateZero() };
        AZ::Transform m_currentTransform{ AZ::Transform::CreateIdentity() };
    };
} // namespace RobotecSpectatorCamera
