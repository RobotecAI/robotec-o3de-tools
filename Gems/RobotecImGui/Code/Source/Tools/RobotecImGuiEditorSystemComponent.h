
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/RobotecImGuiSystemComponent.h>

namespace RobotecImGui
{
    /// System component for RobotecImGui editor
    class RobotecImGuiEditorSystemComponent
        : public RobotecImGuiSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = RobotecImGuiSystemComponent;
    public:
        AZ_COMPONENT_DECL(RobotecImGuiEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        RobotecImGuiEditorSystemComponent();
        ~RobotecImGuiEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace RobotecImGui
