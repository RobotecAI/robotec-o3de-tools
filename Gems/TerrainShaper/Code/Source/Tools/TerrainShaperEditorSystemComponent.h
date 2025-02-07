
#pragma once

#include <Clients/TerrainShaperSystemComponent.h>

#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

namespace TerrainShaper
{
    /// System component for TerrainShaper editor
    class TerrainShaperEditorSystemComponent
        : public TerrainShaperSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = TerrainShaperSystemComponent;
    public:
        AZ_COMPONENT_DECL(TerrainShaperEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        TerrainShaperEditorSystemComponent();
        ~TerrainShaperEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AzToolsFramework::EditorEventsBus overrides ...
        void NotifyRegisterViews() override;
    };
} // namespace TerrainShaper
