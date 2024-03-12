
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/RobotecSplineToolsSystemComponent.h>

namespace RobotecSplineTools
{
    /// System component for RobotecSplineTools editor
    class RobotecSplineToolsEditorSystemComponent
        : public RobotecSplineToolsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = RobotecSplineToolsSystemComponent;
    public:
        AZ_COMPONENT_DECL(RobotecSplineToolsEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        RobotecSplineToolsEditorSystemComponent();
        ~RobotecSplineToolsEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace RobotecSplineTools
