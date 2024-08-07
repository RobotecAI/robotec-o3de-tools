
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/RobotecRecordingToolsSystemComponent.h>

namespace RobotecRecordingTools
{
    /// System component for RobotecRecordingTools editor
    class RobotecRecordingToolsEditorSystemComponent
        : public RobotecRecordingToolsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = RobotecRecordingToolsSystemComponent;

    public:
        AZ_COMPONENT_DECL(RobotecRecordingToolsEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        RobotecRecordingToolsEditorSystemComponent();
        ~RobotecRecordingToolsEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace RobotecRecordingTools
