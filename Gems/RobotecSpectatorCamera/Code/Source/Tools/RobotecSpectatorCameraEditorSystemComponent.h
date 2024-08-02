
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/RobotecSpectatorCameraSystemComponent.h>

namespace RobotecSpectatorCamera
{
    /// System component for RobotecSpectatorCamera editor
    class RobotecSpectatorCameraEditorSystemComponent
        : public RobotecSpectatorCameraSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = RobotecSpectatorCameraSystemComponent;

    public:
        AZ_COMPONENT_DECL(RobotecSpectatorCameraEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        RobotecSpectatorCameraEditorSystemComponent();
        ~RobotecSpectatorCameraEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace RobotecSpectatorCamera
