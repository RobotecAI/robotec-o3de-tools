
#include "RandomizeUtilsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <RandomizeUtils/RandomizeUtilsTypeIds.h>



namespace RandomizeUtils
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(RandomizeUtilsModuleInterface,
        "RandomizeUtilsModuleInterface", RandomizeUtilsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(RandomizeUtilsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(RandomizeUtilsModuleInterface, AZ::SystemAllocator);

    RandomizeUtilsModuleInterface::RandomizeUtilsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            });
    }

    AZ::ComponentTypeList RandomizeUtilsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
        };
    }
} // namespace RandomizeUtils
