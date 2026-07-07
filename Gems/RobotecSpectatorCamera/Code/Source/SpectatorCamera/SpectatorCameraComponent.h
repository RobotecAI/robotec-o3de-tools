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
        float GetOrbitRadius() const override;
        void SetOrbitRadius(const float orbitRadius) override;
        bool GetRequireRmbThirdPerson() const override;
        void SetRequireRmbThirdPerson(const bool requireRmb) override;
        bool GetRequireRmbFreeFlying() const override;
        void SetRequireRmbFreeFlying(const bool requireRmb) override;

        static bool ShouldRotateOnMouse(const SpectatorCameraConfiguration& configuration, bool isRightMouseButtonPressed);

        //! Camera offset relative to the look-at point for the given orbit angles and radius.
        static AZ::Vector3 OrbitOffsetFromAngles(float yaw, float pitch, float radius);

        //! Inverse of OrbitOffsetFromAngles (direction only); pitch is clamped to the orbit limit.
        static void OrbitAnglesFromOffset(const AZ::Vector3& offset, float& yaw, float& pitch);

    private:
        static bool RequiresRightMouseButton(const SpectatorCameraConfiguration& configuration);

        //! Seed yaw/pitch so third-person starts along the authored camera -> look-at ray.
        void SeedOrbitAnglesFromAuthoredTransform();

        void MouseEvent(const AzFramework::InputChannel& inputChannel);
        void KeyboardEvent(const AzFramework::InputChannel& inputChannel);

        AZ::Vector2 GetCurrentMousePosition() const;
        void RotateCameraOnMouse(const AZ::Vector2& mouseDelta);
        void ZoomOrbit(float radiusDelta);
        void ToggleCameraMode();

        static constexpr float scrollValueDivider = 1200.0f;
        static constexpr float pitchDegLimit = 87.0f;
        // Per-input-event keyboard orbit steps: yaw/pitch in radians, zoom in meters.
        static constexpr float orbitKeyboardYawStep = 0.02f;
        static constexpr float orbitKeyboardPitchStep = 0.02f;
        static constexpr float orbitKeyboardZoomStep = 0.1f;

        SpectatorCameraConfiguration m_configuration;
        float m_pitch{ 0.5f };
        float m_yaw{ 0.0f };
        bool m_isRightMouseButtonPressed{ false };
        bool m_ignoreNextMovement{ false };
        bool m_centerTheCursor{ false };
        AZ::Vector2 m_initialMousePosition{ AZ::Vector2::CreateZero() };
        AZ::Vector2 m_lastMousePosition{ AZ::Vector2::CreateZero() };
        AZ::Vector3 m_movement{ AZ::Vector3::CreateZero() };
        AZ::Vector2 m_rotation{ AZ::Vector2::CreateZero() };
        AZ::Transform m_currentTransform{ AZ::Transform::CreateIdentity() };
    };
} // namespace RobotecSpectatorCamera
