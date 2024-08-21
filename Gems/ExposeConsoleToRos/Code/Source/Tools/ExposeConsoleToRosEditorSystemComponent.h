
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ExposeConsoleToRosSystemComponent.h>

namespace ExposeConsoleToRos
{
    /// System component for ExposeConsoleToRos editor
    class ExposeConsoleToRosEditorSystemComponent
        : public ExposeConsoleToRosSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ExposeConsoleToRosSystemComponent;

    public:
        AZ_COMPONENT_DECL(ExposeConsoleToRosEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ExposeConsoleToRosEditorSystemComponent();
        ~ExposeConsoleToRosEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ExposeConsoleToRos
