
#include "ImGuizmoEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ImGuizmo/ImGuizmoTypeIds.h>

namespace ImGuizmo
{
    AZ_COMPONENT_IMPL(
        ImGuizmoEditorSystemComponent, "ImGuizmoEditorSystemComponent", ImGuizmoEditorSystemComponentTypeId, BaseSystemComponent);

    void ImGuizmoEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImGuizmoEditorSystemComponent, ImGuizmoSystemComponent>()->Version(0);
        }
    }

    ImGuizmoEditorSystemComponent::ImGuizmoEditorSystemComponent() = default;

    ImGuizmoEditorSystemComponent::~ImGuizmoEditorSystemComponent() = default;

    void ImGuizmoEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ImGuizmoEditorService"));
    }

    void ImGuizmoEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ImGuizmoEditorService"));
    }

    void ImGuizmoEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ImGuizmoEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    // EditorEntityContextNotificationBus overrides
    void ImGuizmoEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        BaseSystemComponent::Activate();
    }
    void ImGuizmoEditorSystemComponent::OnStopPlayInEditor()
    {
        BaseSystemComponent::Deactivate();
    }

    void ImGuizmoEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void ImGuizmoEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

} // namespace ImGuizmo
