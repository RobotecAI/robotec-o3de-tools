
#include "FPSProfilerModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <Clients/FPSProfilerComponent.h>

namespace FPSProfiler
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(FPSProfilerModuleInterface, "FPSProfilerModuleInterface", FPSProfilerModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(FPSProfilerModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(FPSProfilerModuleInterface, AZ::SystemAllocator);

    FPSProfilerModuleInterface::FPSProfilerModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                FPSProfilerComponent::CreateDescriptor(),
            });
    }

    /**
     * Add required SystemComponents to the SystemEntity.
     * Non-SystemComponents should not be added here
     */
    AZ::ComponentTypeList FPSProfilerModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{};
    }
} // namespace FPSProfiler
