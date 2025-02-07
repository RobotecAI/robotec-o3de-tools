
#include "TerrainShaperModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <TerrainShaper/TerrainShaperTypeIds.h>

#include <Clients/TerrainShaperSystemComponent.h>

namespace TerrainShaper
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(TerrainShaperModuleInterface,
        "TerrainShaperModuleInterface", TerrainShaperModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(TerrainShaperModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(TerrainShaperModuleInterface, AZ::SystemAllocator);

    TerrainShaperModuleInterface::TerrainShaperModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            TerrainShaperSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList TerrainShaperModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<TerrainShaperSystemComponent>(),
        };
    }
} // namespace TerrainShaper
