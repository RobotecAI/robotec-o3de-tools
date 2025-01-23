#include "SpectatorCameraComponent.h"

#include "AzCore/Settings/SettingsRegistry.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Input/Devices/Keyboard/InputDeviceKeyboard.h>
#include <AzFramework/Input/Devices/Mouse/InputDeviceMouse.h>

namespace RobotecSpectatorCamera
{
    constexpr AZStd::string_view CenterTheCursorConfigurationKey = "/O3DE/SpectatorCamera/MoveCursorToTheCenter";

    SpectatorCameraComponent::SpectatorCameraComponent(
        const RobotecSpectatorCamera::SpectatorCameraConfiguration& spectatorCameraConfiguration)
        : m_configuration(spectatorCameraConfiguration)
    {
    }

    void SpectatorCameraComponent::Reflect(AZ::ReflectContext* context)
    {
        SpectatorCameraConfiguration::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SpectatorCameraComponent, AZ::Component>()->Version(0)->Field(
                "Configuration", &SpectatorCameraComponent::m_configuration);
        }
    }

    void SpectatorCameraComponent::Activate()
    {
        RobotecSpectatorCameraRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        AzFramework::InputChannelEventListener::Connect();

        AZ::TransformBus::EventResult(m_currentTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        auto* registry = AZ::SettingsRegistry::Get();
        AZ_Assert(registry, "No Registry available");
        if (registry)
        {
            registry->Get(m_centerTheCursor, CenterTheCursorConfigurationKey);
        }
    }

    void SpectatorCameraComponent::Deactivate()
    {
        RobotecSpectatorCameraRequestBus::Handler::BusDisconnect(GetEntityId());
        AZ::TickBus::Handler::BusDisconnect();
        AzFramework::InputChannelEventListener::Disconnect();
    }

    void SpectatorCameraComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_configuration.m_cameraMode == CameraMode::ThirdPerson)
        {
            // initial position of the camera, 'behind' the target
            AZ::Vector3 cameraLocalPosition{ -m_orbitRadius, 0.0f, 0.0f };
            // calculation of the rotations using the m_yaw and m_pitch values (which correspond to the mouse movement)
            const AZ::Quaternion yawMouseRotation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(1.0f), -m_yaw);
            const AZ::Quaternion pitchMouseRotation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisY(1.0f), m_pitch);
            const AZ::Quaternion finalRotation = yawMouseRotation * pitchMouseRotation;
            // local camera position after the mouse rotations
            cameraLocalPosition = finalRotation.TransformVector(cameraLocalPosition);
            AZ::Transform targetWorldTM = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(targetWorldTM, m_configuration.m_lookAtTarget, &AZ::TransformBus::Events::GetWorldTM);
            AZ::Vector3 currentTranslation = targetWorldTM.GetTranslation();
            // change the target's translation to apply the vertical offset
            const float verticalValueWithOffset = currentTranslation.GetZ() + m_configuration.m_verticalOffset;
            currentTranslation.SetZ(verticalValueWithOffset);
            targetWorldTM.SetTranslation(currentTranslation);
            if (m_configuration.m_followTargetRotation)
            {
                // calculation of the final transform that follows the target's rotation - without mouse input the camera always looks at
                // the same target point
                m_currentTransform =
                    AZ::Transform::CreateLookAt((targetWorldTM.TransformPoint(cameraLocalPosition)), targetWorldTM.GetTranslation());
            }
            else
            {
                // calculation of the final transform that doesn't follow the target's rotation - without mouse input the camera position is
                // const, but the target can change its position in relation to the camera
                m_currentTransform =
                    AZ::Transform::CreateLookAt((targetWorldTM.GetTranslation() + cameraLocalPosition), targetWorldTM.GetTranslation());
            }
        }
        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, m_currentTransform);
    }

    bool SpectatorCameraComponent::OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputDeviceId& deviceId = inputChannel.GetInputDevice().GetInputDeviceId();

        if (AzFramework::InputDeviceMouse::IsMouseDevice(deviceId))
        {
            MouseEvent(inputChannel);
        }

        if (AzFramework::InputDeviceKeyboard::IsKeyboardDevice(deviceId))
        {
            KeyboardEvent(inputChannel);
        }

        return false;
    }

    void SpectatorCameraComponent::MouseEvent(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputChannelId& channelId = inputChannel.GetInputChannelId();

        if (channelId == AzFramework::InputDeviceMouse::Movement::Z && m_configuration.m_cameraMode == CameraMode::ThirdPerson)
        {
            m_orbitRadius = AZStd::clamp(
                m_orbitRadius - (inputChannel.GetValue() / scrollValueDivider),
                SpectatorCameraConfiguration::OrbitRadiusMin,
                SpectatorCameraConfiguration::OrbitRadiusMax);
        }

        if (channelId == AzFramework::InputDeviceMouse::Button::Right && m_configuration.m_cameraMode == CameraMode::ThirdPerson)
        {
            AzFramework::SystemCursorState currentCursorState;
            AzFramework::InputSystemCursorRequestBus::EventResult(
                currentCursorState, AzFramework::InputDeviceMouse::Id, &AzFramework::InputSystemCursorRequests::GetSystemCursorState);
            const bool isConstrained = // flag to do not lose information whether the cursor is constrained or not
                (currentCursorState == AzFramework::SystemCursorState::ConstrainedAndHidden ||
                 currentCursorState == AzFramework::SystemCursorState::ConstrainedAndVisible);
            if (inputChannel.IsStateBegan())
            {
                m_isRightMouseButtonPressed = true;

                // Capture the initial mouse position and cursor state
                m_initialMousePosition = GetCurrentMousePosition();
                m_ignoreNextMovement = true; // Flag to ignore the next mouse movement preventing jump on re-centering
                AzFramework::InputSystemCursorRequestBus::Event(
                    AzFramework::InputDeviceMouse::Id,
                    &AzFramework::InputSystemCursorRequests::SetSystemCursorState,
                    isConstrained ? AzFramework::SystemCursorState::ConstrainedAndHidden
                                  : AzFramework::SystemCursorState::UnconstrainedAndHidden);
            }
            else if (inputChannel.IsStateEnded())
            {
                m_isRightMouseButtonPressed = false;

                if (m_centerTheCursor)
                {
                    // Restore the cursor's original position
                    AzFramework::InputSystemCursorRequestBus::Event(
                        AzFramework::InputDeviceMouse::Id,
                        &AzFramework::InputSystemCursorRequests::SetSystemCursorPositionNormalized,
                        m_initialMousePosition);

                    // Update m_lastMousePosition to the restored position to prevent the jump on the next rotation start
                    m_lastMousePosition = m_initialMousePosition;
                }
                AzFramework::InputSystemCursorRequestBus::Event(
                    AzFramework::InputDeviceMouse::Id,
                    &AzFramework::InputSystemCursorRequests::SetSystemCursorState,
                    isConstrained ? AzFramework::SystemCursorState::ConstrainedAndVisible
                                  : AzFramework::SystemCursorState::UnconstrainedAndVisible);
            }
        }

        if ((m_isRightMouseButtonPressed || m_configuration.m_cameraMode == CameraMode::FreeFlying) &&
            (channelId == AzFramework::InputDeviceMouse::Movement::X || channelId == AzFramework::InputDeviceMouse::Movement::Y))
        {
            if (m_ignoreNextMovement)
            {
                m_ignoreNextMovement = false; // Reset the flag after ignoring one movement
                m_lastMousePosition = GetCurrentMousePosition(); // Update the last position to current to avoid the jump
                return; // Do not process this movement
            }

            AZ::Vector2 currentMousePosition = GetCurrentMousePosition();
            AZ::Vector2 mouseDelta = currentMousePosition - m_lastMousePosition;

            if (m_centerTheCursor)
            {
                const auto center = AZ::Vector2(0.5f, 0.5f);

                // Recenter the cursor to avoid edge constraints
                AzFramework::InputSystemCursorRequestBus::Event(
                    AzFramework::InputDeviceMouse::Id, &AzFramework::InputSystemCursorRequests::SetSystemCursorPositionNormalized, center);

                // Then update m_lastMousePosition accordingly
                m_lastMousePosition = center;
            }
            else
            {
                m_lastMousePosition = currentMousePosition;
            }

            RotateCameraOnMouse(mouseDelta);
        }
    }

    void SpectatorCameraComponent::KeyboardEvent(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputChannelId& channelId = inputChannel.GetInputChannelId();
        if (inputChannel.IsStateBegan())
        {
            if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericC)
            {
                ToggleCameraMode();
            }
        }

        if (m_configuration.m_cameraMode == CameraMode::FreeFlying)
        {
            AZ::Vector3 localCamPositionChange = AZ::Vector3::CreateZero();

            if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericW)
            {
                localCamPositionChange += AZ::Vector3::CreateAxisY() * m_configuration.m_cameraSpeed;
            }
            if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericS)
            {
                localCamPositionChange -= AZ::Vector3::CreateAxisY() * m_configuration.m_cameraSpeed;
            }
            if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericA)
            {
                localCamPositionChange -= AZ::Vector3::CreateAxisX() * m_configuration.m_cameraSpeed;
            }
            if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericD)
            {
                localCamPositionChange += AZ::Vector3::CreateAxisX() * m_configuration.m_cameraSpeed;
            }
            if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericQ)
            {
                localCamPositionChange -= AZ::Vector3::CreateAxisZ() * m_configuration.m_cameraSpeed;
            }
            if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericE)
            {
                localCamPositionChange += AZ::Vector3::CreateAxisZ() * m_configuration.m_cameraSpeed;
            }

            m_currentTransform = m_currentTransform * AZ::Transform::CreateTranslation(localCamPositionChange);
        }
    }

    AZ::Vector2 SpectatorCameraComponent::GetCurrentMousePosition() const
    {
        AZ::Vector2 mousePosition = AZ::Vector2::CreateZero();

        // Query the input system for the current mouse position
        AzFramework::InputSystemCursorRequestBus::EventResult(
            mousePosition, AzFramework::InputDeviceMouse::Id, &AzFramework::InputSystemCursorRequests::GetSystemCursorPositionNormalized);

        return mousePosition;
    }

    void SpectatorCameraComponent::RotateCameraOnMouse(const AZ::Vector2& mouseDelta)
    {
        if (m_configuration.m_cameraMode == CameraMode::ThirdPerson)
        {
            m_yaw += mouseDelta.GetX() * m_configuration.m_mouseSensitivity;
            if (AZStd::abs(m_yaw) >= AZ::DegToRad(360.0f))
            {
                m_yaw = 0.0f;
            }
            // block pitch to go above 87 degrees - this is done to avoid strange camera jumps that start to appear at around 87.5-88
            // degrees
            if (float newPitch = m_pitch + (mouseDelta.GetY() * m_configuration.m_mouseSensitivity);
                AZStd::abs(newPitch) < AZ::DegToRad(pitchDegLimit))
            {
                m_pitch += mouseDelta.GetY() * m_configuration.m_mouseSensitivity;
            }
        }

        if (m_configuration.m_cameraMode == CameraMode::FreeFlying)
        {
            float yawRotation = -mouseDelta.GetX() * m_configuration.m_mouseSensitivity; // Inverted X for reversed yaw rotation
            float pitchRotation = -mouseDelta.GetY() * m_configuration.m_mouseSensitivity; // Inverted Y for pitch, adjust as needed

            const AZ::Quaternion currentRotation = m_currentTransform.GetRotation();

            AZ::Quaternion yawQuat = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), yawRotation);

            // Apply yaw rotation to current camera orientation
            AZ::Quaternion tempRotation = yawQuat * currentRotation;

            // Calculate pitch rotation around the camera's local X-axis after applying yaw
            // This ensures that pitch adjustments are made relative to the camera's adjusted orientation
            const AZ::Vector3 localXAxis = tempRotation.TransformVector(AZ::Vector3::CreateAxisX());
            const auto pitchQuat = AZ::Quaternion::CreateFromAxisAngle(localXAxis, pitchRotation);

            // Apply pitch rotation
            auto newRotation = pitchQuat * tempRotation;
            newRotation.Normalize(); // Normalize the quaternion to prevent drift and ensure no numerical instability

            // Update the camera's offset matrix with the new orientation, keeping the position unchanged
            const AZ::Vector3 currentPosition = m_currentTransform.GetTranslation();
            m_currentTransform = AZ::Transform::CreateFromQuaternionAndTranslation(newRotation, currentPosition);
        }
    }

    void SpectatorCameraComponent::ToggleCameraMode()
    {
        // RMB should be handled only in the ThirdPerson mode. This line fixes problem with such combination:
        // RMB pressed -> Key C pressed -> FreeFlying -> RMB released -> Key C pressed -> ThirdPerson -> Camera rotates without pressing RMB
        m_isRightMouseButtonPressed = false;
        if (m_configuration.m_cameraMode == CameraMode::ThirdPerson)
        {
            m_configuration.m_cameraMode = CameraMode::FreeFlying;
            return;
        }
        m_configuration.m_cameraMode = CameraMode::ThirdPerson;
    }

    CameraMode SpectatorCameraComponent::GetCameraMode() const
    {
        return m_configuration.m_cameraMode;
    }

    void SpectatorCameraComponent::SetCameraMode(const CameraMode cameraMode)
    {
        m_configuration.m_cameraMode = cameraMode;
    }

    float SpectatorCameraComponent::GetMouseSensitivity() const
    {
        return m_configuration.m_mouseSensitivity;
    }

    void SpectatorCameraComponent::SetMouseSensitivity(const float mouseSensitivity)
    {
        m_configuration.m_mouseSensitivity = mouseSensitivity;
    }

    float SpectatorCameraComponent::GetCameraSpeed() const
    {
        return m_configuration.m_cameraSpeed;
    }

    void SpectatorCameraComponent::SetCameraSpeed(const float cameraSpeed)
    {
        m_configuration.m_cameraSpeed = cameraSpeed;
    }

    bool SpectatorCameraComponent::GetFollowTargetRotation() const
    {
        return m_configuration.m_followTargetRotation;
    }

    void SpectatorCameraComponent::SetFollowTargetRotation(const bool followTargetRotation)
    {
        m_configuration.m_followTargetRotation = followTargetRotation;
    }

    float SpectatorCameraComponent::GetVerticalOffset() const
    {
        return m_configuration.m_verticalOffset;
    }

    void SpectatorCameraComponent::SetVerticalOffset(const float verticalOffset)
    {
        m_configuration.m_verticalOffset = verticalOffset;
    }
} // namespace RobotecSpectatorCamera
