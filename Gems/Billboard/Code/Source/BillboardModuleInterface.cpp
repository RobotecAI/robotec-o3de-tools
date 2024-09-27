
#include "BillboardModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include "Billboard/BillboardComponent.h"
#include <Billboard/BillboardTypeIds.h>

namespace BillboardComponent
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(BillboardModuleInterface, "BillboardModuleInterface", BillboardModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(BillboardModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(BillboardModuleInterface, AZ::SystemAllocator);

    BillboardModuleInterface::BillboardModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                BillboardComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList BillboardModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{};
    }
} // namespace BillboardComponent
