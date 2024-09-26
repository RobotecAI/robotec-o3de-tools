
#include "CsvSpawnerModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <CsvSpawner/CsvSpawnerComponent.h>
#include <CsvSpawner/CsvSpawnerTypeIds.h>

namespace CsvSpawner
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(CsvSpawnerModuleInterface, "CsvSpawnerModuleInterface", CsvSpawnerModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(CsvSpawnerModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(CsvSpawnerModuleInterface, AZ::SystemAllocator);

    CsvSpawnerModuleInterface::CsvSpawnerModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                CsvSpawnerComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList CsvSpawnerModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{};
    }
} // namespace CsvSpawner
