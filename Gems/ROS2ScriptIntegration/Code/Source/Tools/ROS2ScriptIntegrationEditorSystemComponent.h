
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ROS2ScriptIntegrationSystemComponent.h>

namespace ROS2ScriptIntegration
{
    /// System component for ROS2ScriptIntegration editor
    class ROS2ScriptIntegrationEditorSystemComponent
        : public ROS2ScriptIntegrationSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ROS2ScriptIntegrationSystemComponent;
    public:
        AZ_COMPONENT_DECL(ROS2ScriptIntegrationEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2ScriptIntegrationEditorSystemComponent();
        ~ROS2ScriptIntegrationEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2ScriptIntegration
