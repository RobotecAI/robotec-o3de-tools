
#include "PhysicalTimeToolsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <PhysicalTimeTools/PhysicalTimeToolsTypeIds.h>

#include <Clients/PhysicalTimeToolsSystemComponent.h>

namespace PhysicalTimeTools
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(PhysicalTimeToolsModuleInterface,
        "PhysicalTimeToolsModuleInterface", PhysicalTimeToolsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(PhysicalTimeToolsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(PhysicalTimeToolsModuleInterface, AZ::SystemAllocator);

    PhysicalTimeToolsModuleInterface::PhysicalTimeToolsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            PhysicalTimeToolsSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList PhysicalTimeToolsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<PhysicalTimeToolsSystemComponent>(),
        };
    }
} // namespace PhysicalTimeTools
