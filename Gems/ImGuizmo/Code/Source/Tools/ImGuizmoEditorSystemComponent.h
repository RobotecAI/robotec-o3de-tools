
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ImGuizmoSystemComponent.h>

namespace ImGuizmo
{
    /// System component for ImGuizmo editor
    class ImGuizmoEditorSystemComponent
        : public ImGuizmoSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ImGuizmoSystemComponent;
    public:
        AZ_COMPONENT_DECL(ImGuizmoEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ImGuizmoEditorSystemComponent();
        ~ImGuizmoEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ImGuizmo
