
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/SmoothingSystemComponent.h>

namespace Smoothing
{
    /// System component for Smoothing editor
    class SmoothingEditorSystemComponent
        : public SmoothingSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = SmoothingSystemComponent;

    public:
        AZ_COMPONENT_DECL(SmoothingEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        SmoothingEditorSystemComponent();
        ~SmoothingEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace Smoothing
