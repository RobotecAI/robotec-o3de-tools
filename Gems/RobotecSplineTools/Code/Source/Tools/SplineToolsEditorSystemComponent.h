
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/SplineToolsSystemComponent.h>

namespace SplineTools
{
    /// System component for SplineTools editor
    class SplineToolsEditorSystemComponent
        : public SplineToolsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = SplineToolsSystemComponent;
    public:
        AZ_COMPONENT_DECL(SplineToolsEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        SplineToolsEditorSystemComponent();
        ~SplineToolsEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace SplineTools
