#include "SpectatorCameraConfiguration.h"
#include <AzCore/Serialization/EditContext.h>

namespace RobotecSpectatorCamera
{
    void SpectatorCameraConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SpectatorCameraConfiguration>()
                ->Version(1)
                ->Field("LookAtTarget", &SpectatorCameraConfiguration::m_lookAtTarget)
                ->Field("FollowTargetRotation", &SpectatorCameraConfiguration::m_followTargetRotation)
                ->Field("MouseSensitivity", &SpectatorCameraConfiguration::m_mouseSensitivity)
                ->Field("CameraSpeed", &SpectatorCameraConfiguration::m_cameraSpeed)
                ->Field("VerticalOffset", &SpectatorCameraConfiguration::m_verticalOffset)
                ->Field("OrbitRadius", &SpectatorCameraConfiguration::m_orbitRadius)
                ->Field("RequireRmbThirdPerson", &SpectatorCameraConfiguration::m_requireRmbThirdPerson)
                ->Field("RequireRmbFreeFlying", &SpectatorCameraConfiguration::m_requireRmbFreeFlying)
                ->Field("SeedOrbitFromPlacement", &SpectatorCameraConfiguration::m_seedOrbitFromPlacement);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SpectatorCameraConfiguration>("SpectatorCameraConfiguration", "SpectatorCameraConfiguration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraConfiguration::m_lookAtTarget,
                        "Look-at target",
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
                    ->Attribute(AZ::Edit::Attributes::Max, CameraSpeedMax)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraConfiguration::m_verticalOffset,
                        "Vertical offset",
                        "Vertical offset to change the point of the target which is used to calculate LookAt transform")
                    ->Attribute(AZ::Edit::Attributes::Min, VerticalOffsetMin)
                    ->Attribute(AZ::Edit::Attributes::Max, VerticalOffsetMax)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider,
                        &SpectatorCameraConfiguration::m_orbitRadius,
                        "Orbit radius",
                        "Initial third-person distance from the look-at target (also adjustable at runtime via scroll or W/S)")
                    ->Attribute(AZ::Edit::Attributes::Min, OrbitRadiusMin)
                    ->Attribute(AZ::Edit::Attributes::Max, OrbitRadiusMax)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraConfiguration::m_requireRmbThirdPerson,
                        "Third-person RMB",
                        "When enabled, mouse look in third-person mode only applies while the right mouse button is held")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraConfiguration::m_requireRmbFreeFlying,
                        "Free-flying RMB",
                        "When enabled, mouse look in free-flying mode only applies while the right mouse button is held")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraConfiguration::m_seedOrbitFromPlacement,
                        "Start at placement",
                        "When enabled, the initial orbit angles derive from the entity's placed transform; otherwise the camera starts "
                        "behind the target");
            }
        }
    }
} // namespace RobotecSpectatorCamera
