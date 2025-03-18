
#include "ImGuiProviderEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ImGuiProvider/ImGuiProviderTypeIds.h>

namespace ImGuiProvider
{
    AZ_COMPONENT_IMPL(
        ImGuiProviderEditorSystemComponent,
        "ImGuiProviderEditorSystemComponent",
        ImGuiProviderEditorSystemComponentTypeId,
        BaseSystemComponent);

    void ImGuiProviderEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImGuiProviderEditorSystemComponent, ImGuiProviderSystemComponent>()->Version(0);
        }
    }

    ImGuiProviderEditorSystemComponent::ImGuiProviderEditorSystemComponent() = default;

    ImGuiProviderEditorSystemComponent::~ImGuiProviderEditorSystemComponent() = default;

    void ImGuiProviderEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ImGuiProviderEditorService"));
    }

    void ImGuiProviderEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ImGuiProviderEditorService"));
    }

    void ImGuiProviderEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ImGuiProviderEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ImGuiProviderEditorSystemComponent::Activate()
    {
        ImGuiProviderSystemComponent::Activate();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void ImGuiProviderEditorSystemComponent::Deactivate()
    {
        ImGuiProviderSystemComponent::Deactivate();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void ImGuiProviderEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        ImGuiProviderSystemComponent::SetEditorPlayModeState(true);
    }
    void ImGuiProviderEditorSystemComponent::OnStopPlayInEditor()
    {
        ImGuiProviderSystemComponent::SetEditorPlayModeState(false);
    }

} // namespace ImGuiProvider
