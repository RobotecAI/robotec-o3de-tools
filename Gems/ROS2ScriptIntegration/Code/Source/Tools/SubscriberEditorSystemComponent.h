
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/SubscriberSystemComponent.h>

namespace ROS2ScriptIntegration
{
    /// System component for ROS2ScriptIntegration editor
    class SubscriberEditorSystemComponent
        : public SubscriberSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = SubscriberSystemComponent;

    public:
        AZ_COMPONENT_DECL(SubscriberEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        SubscriberEditorSystemComponent();
        ~SubscriberEditorSystemComponent();

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
