
#include "GeoJSONSpawnerModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

#include <Clients/GeoJSONSpawnerSystemComponent.h>

namespace GeoJSONSpawner
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(GeoJSONSpawnerModuleInterface,
        "GeoJSONSpawnerModuleInterface", GeoJSONSpawnerModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(GeoJSONSpawnerModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(GeoJSONSpawnerModuleInterface, AZ::SystemAllocator);

    GeoJSONSpawnerModuleInterface::GeoJSONSpawnerModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            GeoJSONSpawnerSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList GeoJSONSpawnerModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<GeoJSONSpawnerSystemComponent>(),
        };
    }
} // namespace GeoJSONSpawner
