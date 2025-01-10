
#include <AzCore/Serialization/SerializeContext.h>
#include "ImGuizmoGemEditorSystemComponent.h"

#include <ImGuizmoGem/ImGuizmoGemTypeIds.h>

namespace ImGuizmoGem
{
    AZ_COMPONENT_IMPL(ImGuizmoGemEditorSystemComponent, "ImGuizmoGemEditorSystemComponent",
        ImGuizmoGemEditorSystemComponentTypeId, BaseSystemComponent);

    void ImGuizmoGemEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImGuizmoGemEditorSystemComponent, ImGuizmoGemSystemComponent>()
                ->Version(0);
        }
    }

    ImGuizmoGemEditorSystemComponent::ImGuizmoGemEditorSystemComponent() = default;

    ImGuizmoGemEditorSystemComponent::~ImGuizmoGemEditorSystemComponent() = default;

    void ImGuizmoGemEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ImGuizmoGemEditorService"));
    }

    void ImGuizmoGemEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ImGuizmoGemEditorService"));
    }

    void ImGuizmoGemEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ImGuizmoGemEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ImGuizmoGemEditorSystemComponent::Activate()
    {
        ImGuizmoGemSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ImGuizmoGemEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ImGuizmoGemSystemComponent::Deactivate();
    }

} // namespace ImGuizmoGem
