
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/CsvSpawnerSystemComponent.h>

namespace CsvSpawner
{
    /// System component for CsvSpawner editor
    class CsvSpawnerEditorSystemComponent
        : public CsvSpawnerSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = CsvSpawnerSystemComponent;

    public:
        AZ_COMPONENT_DECL(CsvSpawnerEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        CsvSpawnerEditorSystemComponent();
        ~CsvSpawnerEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace CsvSpawner
