#include "SpectatorCameraConfiguration.h"
#include <AzCore/Serialization/EditContext.h>

namespace RobotecSpectatorCamera
{
    void SpectatorCameraConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SpectatorCameraConfiguration>()
                ->Version(0)
                ->Field("LookAtTarget", &SpectatorCameraConfiguration::m_lookAtTarget)
                ->Field("FollowTargetRotation", &SpectatorCameraConfiguration::m_followTargetRotation)
                ->Field("MouseSensitivity", &SpectatorCameraConfiguration::m_mouseSensitivity)
                ->Field("CameraSpeed", &SpectatorCameraConfiguration::m_cameraSpeed);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SpectatorCameraConfiguration>("SpectatorCameraConfiguration", "SpectatorCameraConfiguration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraConfiguration::m_lookAtTarget,
                        "Look at target entity ID",
                        "Look at target entity ID")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraConfiguration::m_followTargetRotation,
                        "Follow the target's rotation",
                        "Follow the target's rotation")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider,
                        &SpectatorCameraConfiguration::m_mouseSensitivity,
                        "Mouse sensitivity",
                        "Mouse sensitivity")
                    ->Attribute(AZ::Edit::Attributes::Min, SensitivityMin)
                    ->Attribute(AZ::Edit::Attributes::Max, SensitivityMax)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider, &SpectatorCameraConfiguration::m_cameraSpeed, "Camera speed", "Camera speed")
                    ->Attribute(AZ::Edit::Attributes::Min, CameraSpeedMin)
                    ->Attribute(AZ::Edit::Attributes::Max, CameraSpeedMax);
            }
        }
    }
} // namespace RobotecSpectatorCamera
