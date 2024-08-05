
#include "RobotecSpectatorCameraEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

namespace RobotecSpectatorCamera
{
    AZ_COMPONENT_IMPL(
        RobotecSpectatorCameraEditorSystemComponent,
        "RobotecSpectatorCameraEditorSystemComponent",
        RobotecSpectatorCameraEditorSystemComponentTypeId,
        BaseSystemComponent);

    void RobotecSpectatorCameraEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecSpectatorCameraEditorSystemComponent, RobotecSpectatorCameraSystemComponent>()->Version(0);
        }
    }

    RobotecSpectatorCameraEditorSystemComponent::RobotecSpectatorCameraEditorSystemComponent() = default;

    RobotecSpectatorCameraEditorSystemComponent::~RobotecSpectatorCameraEditorSystemComponent() = default;

    void RobotecSpectatorCameraEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("RobotecSpectatorCameraEditorService"));
    }

    void RobotecSpectatorCameraEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("RobotecSpectatorCameraEditorService"));
    }

    void RobotecSpectatorCameraEditorSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void RobotecSpectatorCameraEditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void RobotecSpectatorCameraEditorSystemComponent::Activate()
    {
        RobotecSpectatorCameraSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void RobotecSpectatorCameraEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        RobotecSpectatorCameraSystemComponent::Deactivate();
    }

} // namespace RobotecSpectatorCamera
