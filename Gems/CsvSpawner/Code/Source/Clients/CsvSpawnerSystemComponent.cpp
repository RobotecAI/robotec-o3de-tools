
#include "CsvSpawnerSystemComponent.h"

#include <CsvSpawner/CsvSpawnerTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace CsvSpawner
{
    AZ_COMPONENT_IMPL(CsvSpawnerSystemComponent, "CsvSpawnerSystemComponent", CsvSpawnerSystemComponentTypeId);

    void CsvSpawnerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<CsvSpawnerSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void CsvSpawnerSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("CsvSpawnerService"));
    }

    void CsvSpawnerSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("CsvSpawnerService"));
    }

    void CsvSpawnerSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void CsvSpawnerSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void CsvSpawnerSystemComponent::Init()
    {
    }

    void CsvSpawnerSystemComponent::Activate()
    {
    }

    void CsvSpawnerSystemComponent::Deactivate()
    {
    }
} // namespace CsvSpawner
