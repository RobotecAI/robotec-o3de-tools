
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/PhysicalTimeToolsSystemComponent.h>

namespace PhysicalTimeTools
{
    /// System component for PhysicalTimeTools editor
    class PhysicalTimeToolsEditorSystemComponent
        : public PhysicalTimeToolsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = PhysicalTimeToolsSystemComponent;
    public:
        AZ_COMPONENT_DECL(PhysicalTimeToolsEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        PhysicalTimeToolsEditorSystemComponent();
        ~PhysicalTimeToolsEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace PhysicalTimeTools
