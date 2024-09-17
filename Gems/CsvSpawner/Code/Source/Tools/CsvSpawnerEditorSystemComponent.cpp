
#include "CsvSpawnerEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <CsvSpawner/CsvSpawnerTypeIds.h>

namespace CsvSpawner
{
    AZ_COMPONENT_IMPL(
        CsvSpawnerEditorSystemComponent, "CsvSpawnerEditorSystemComponent", CsvSpawnerEditorSystemComponentTypeId, BaseSystemComponent);

    void CsvSpawnerEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<CsvSpawnerEditorSystemComponent, CsvSpawnerSystemComponent>()->Version(0);
        }
    }

    CsvSpawnerEditorSystemComponent::CsvSpawnerEditorSystemComponent() = default;

    CsvSpawnerEditorSystemComponent::~CsvSpawnerEditorSystemComponent() = default;

    void CsvSpawnerEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("CsvSpawnerEditorService"));
    }

    void CsvSpawnerEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("CsvSpawnerEditorService"));
    }

    void CsvSpawnerEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void CsvSpawnerEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void CsvSpawnerEditorSystemComponent::Activate()
    {
        CsvSpawnerSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void CsvSpawnerEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        CsvSpawnerSystemComponent::Deactivate();
    }

} // namespace CsvSpawner
