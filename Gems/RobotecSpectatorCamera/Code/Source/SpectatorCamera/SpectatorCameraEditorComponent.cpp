#include "SpectatorCameraEditorComponent.h"
#include "SpectatorCameraComponent.h"
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzToolsFramework/API/EditorCameraBus.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

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

    void SpectatorCameraEditorComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        Camera::EditorCameraNotificationBus::Handler::BusConnect();
    }

    void SpectatorCameraEditorComponent::Deactivate()
    {
        Camera::EditorCameraNotificationBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    bool SpectatorCameraEditorComponent::IsActiveViewCamera() const
    {
        AZ::EntityId currentViewEntity;
        Camera::EditorCameraRequests::Bus::BroadcastResult(currentViewEntity, &Camera::EditorCameraRequests::GetCurrentViewEntityId);
        return currentViewEntity == GetEntityId();
    }

    void SpectatorCameraEditorComponent::OnTransformChanged(
        [[maybe_unused]] const AZ::Transform& local, [[maybe_unused]] const AZ::Transform& world)
    {
        // Track the radius only while "Be this camera" is engaged, riding its viewport -> entity transform sync.
        if (IsActiveViewCamera())
        {
            UpdateOrbitRadiusFromCurrentTransform();
        }
    }

    void SpectatorCameraEditorComponent::OnViewportViewEntityChanged(const AZ::EntityId& newViewId)
    {
        // Becoming the active view emits no transform change of its own, so refresh here too.
        if (newViewId == GetEntityId())
        {
            UpdateOrbitRadiusFromCurrentTransform();
        }
    }

    void SpectatorCameraEditorComponent::UpdateOrbitRadiusFromCurrentTransform()
    {
        if (!m_configuration.m_lookAtTarget.IsValid())
        {
            return;
        }

        AZ::Transform cameraWorldTM = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(cameraWorldTM, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        AZ::Transform targetWorldTM = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(targetWorldTM, m_configuration.m_lookAtTarget, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Vector3 lookAtPoint = targetWorldTM.GetTranslation();
        lookAtPoint.SetZ(lookAtPoint.GetZ() + m_configuration.m_verticalOffset);

        const float newRadius = AZStd::clamp(
            (cameraWorldTM.GetTranslation() - lookAtPoint).GetLength(),
            SpectatorCameraConfiguration::OrbitRadiusMin,
            SpectatorCameraConfiguration::OrbitRadiusMax);

        if (!AZ::IsClose(newRadius, m_configuration.m_orbitRadius))
        {
            m_configuration.m_orbitRadius = newRadius;
            AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
                &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay, AzToolsFramework::Refresh_Values);
        }
    }

    void SpectatorCameraEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        AZ_Error("SpectatorCameraEditorComponent", m_configuration.m_lookAtTarget.IsValid(), "LookAtTarget's EntityId is not valid");

        gameEntity->CreateComponent<SpectatorCameraComponent>(m_configuration);
    }

} // namespace RobotecSpectatorCamera
