
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <AzToolsFramework/Entity/EditorEntityContextBus.h>
#include <Clients/ImGuiProviderSystemComponent.h>

namespace ImGuiProvider
{
    /// System component for ImGuiProvider editor
    class ImGuiProviderEditorSystemComponent
        : public ImGuiProviderSystemComponent
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
        using BaseSystemComponent = ImGuiProviderSystemComponent;

    public:
        AZ_COMPONENT_DECL(ImGuiProviderEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ImGuiProviderEditorSystemComponent();
        ~ImGuiProviderEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // EditorEntityContextNotificationBus overrides
        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditor() override;
    };
} // namespace ImGuiProvider
