#include "SpectatorCameraEditorComponent.h"
#include "SpectatorCameraComponent.h"
#include <AzCore/Serialization/EditContext.h>

namespace RobotecSpectatorCamera
{
    void SpectatorCameraEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SpectatorCameraEditorComponent, AzToolsFramework::Components::EditorComponentBase>()->Version(0)->Field(
                "Configuration", &SpectatorCameraEditorComponent::m_configuration);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SpectatorCameraEditorComponent>("Spectator Camera", "Spectator Camera")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "SpectatorCameraEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Robotec Camera System")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SpectatorCameraEditorComponent::m_configuration,
                        "Spectator camera configuration",
                        "Spectator camera configuration");
            }
        }
    }

    void SpectatorCameraEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("CameraService"));
    }

    void SpectatorCameraEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        AZ_Error("SpectatorCameraEditorComponent", m_configuration.m_lookAtTarget.IsValid(), "LookAtTarget's EntityId is not valid");

        gameEntity->CreateComponent<SpectatorCameraComponent>(m_configuration);
    }

} // namespace RobotecSpectatorCamera
