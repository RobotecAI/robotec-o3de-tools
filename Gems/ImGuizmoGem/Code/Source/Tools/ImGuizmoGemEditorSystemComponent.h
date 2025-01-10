
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ImGuizmoGemSystemComponent.h>

namespace ImGuizmoGem
{
    /// System component for ImGuizmoGem editor
    class ImGuizmoGemEditorSystemComponent
        : public ImGuizmoGemSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ImGuizmoGemSystemComponent;
    public:
        AZ_COMPONENT_DECL(ImGuizmoGemEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ImGuizmoGemEditorSystemComponent();
        ~ImGuizmoGemEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ImGuizmoGem
