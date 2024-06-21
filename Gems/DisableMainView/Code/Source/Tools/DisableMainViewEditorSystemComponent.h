
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/DisableMainViewSystemComponent.h>

namespace DisableMainView {
    /// System component for DisableMainView editor
    class DisableMainViewEditorSystemComponent
            : public DisableMainViewSystemComponent, protected AzToolsFramework::EditorEvents::Bus::Handler {
        using BaseSystemComponent = DisableMainViewSystemComponent;
    public:
        AZ_COMPONENT_DECL(DisableMainViewEditorSystemComponent);

        static void Reflect(AZ::ReflectContext *context);

        DisableMainViewEditorSystemComponent();

        ~DisableMainViewEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType &provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType &incompatible);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType &required);

        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType &dependent);

        // AZ::Component
        void Activate() override;

        void Deactivate() override;
    };
} // namespace DisableMainView
