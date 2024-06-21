
#include "DisableMainViewModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <DisableMainView/DisableMainViewTypeIds.h>

#include <Clients/DisableMainViewSystemComponent.h>

namespace DisableMainView
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(DisableMainViewModuleInterface,
        "DisableMainViewModuleInterface", DisableMainViewModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(DisableMainViewModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(DisableMainViewModuleInterface, AZ::SystemAllocator);

    DisableMainViewModuleInterface::DisableMainViewModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            DisableMainViewSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList DisableMainViewModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<DisableMainViewSystemComponent>(),
        };
    }
} // namespace DisableMainView
