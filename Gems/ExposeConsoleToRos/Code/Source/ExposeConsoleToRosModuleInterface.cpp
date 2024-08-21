
#include "ExposeConsoleToRosModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ExposeConsoleToRos/ExposeConsoleToRosTypeIds.h>

#include <Clients/ExposeConsoleToRosSystemComponent.h>

namespace ExposeConsoleToRos
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        ExposeConsoleToRosModuleInterface, "ExposeConsoleToRosModuleInterface", ExposeConsoleToRosModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ExposeConsoleToRosModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ExposeConsoleToRosModuleInterface, AZ::SystemAllocator);

    ExposeConsoleToRosModuleInterface::ExposeConsoleToRosModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ExposeConsoleToRosSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ExposeConsoleToRosModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ExposeConsoleToRosSystemComponent>(),
        };
    }
} // namespace ExposeConsoleToRos
