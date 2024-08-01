
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ROS2PoseControlSystemComponent.h>

namespace ROS2PoseControl
{
    /// System component for ROS2PoseControl editor
    class ROS2PoseControlEditorSystemComponent
        : public ROS2PoseControlSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ROS2PoseControlSystemComponent;

    public:
        AZ_COMPONENT_DECL(ROS2PoseControlEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2PoseControlEditorSystemComponent();
        ~ROS2PoseControlEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2PoseControl
