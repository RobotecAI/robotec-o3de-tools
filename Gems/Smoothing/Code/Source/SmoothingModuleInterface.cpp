
#include "SmoothingModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <Smoothing/SmoothingTypeIds.h>

#include "Clients/SmoothingComponent.h"
#include <Clients/SmoothingSystemComponent.h>

namespace Smoothing
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(SmoothingModuleInterface, "SmoothingModuleInterface", SmoothingModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(SmoothingModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(SmoothingModuleInterface, AZ::SystemAllocator);

    SmoothingModuleInterface::SmoothingModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                SmoothingSystemComponent::CreateDescriptor(),
                SmoothingComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList SmoothingModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<SmoothingSystemComponent>(),
        };
    }
} // namespace Smoothing
