#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

namespace RobotecSpectatorCamera
{
    enum class CameraMode
    {
        ThirdPerson = 0,
        FreeFlying
    };

    struct SpectatorCameraConfiguration
    {
        AZ_TYPE_INFO(SpectatorCameraConfiguration, SpectatorCameraConfigurationTypeId);

        static void Reflect(AZ::ReflectContext* context);

        static constexpr float OrbitRadiusMin = 0.5f;
        static constexpr float OrbitRadiusMax = 50.0f;
        static constexpr float SensitivityMin = 0.5f;
        static constexpr float SensitivityMax = 30.0f;
        static constexpr float CameraSpeedMin = 0.05f;
        static constexpr float CameraSpeedMax = 1.0f;

        AZ::EntityId m_lookAtTarget;
        float m_mouseSensitivity{ 1.0f };
        float m_cameraSpeed{ CameraSpeedMin };
        bool m_followTargetRotation{ true };
        CameraMode m_cameraMode{ CameraMode::ThirdPerson };
    };
} // namespace RobotecSpectatorCamera
