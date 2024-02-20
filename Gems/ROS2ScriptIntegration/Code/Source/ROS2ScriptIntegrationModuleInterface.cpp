
#include "ROS2ScriptIntegrationModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>

#include <Clients/ROS2ScriptIntegrationSystemComponent.h>

namespace ROS2ScriptIntegration
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2ScriptIntegrationModuleInterface,
        "ROS2ScriptIntegrationModuleInterface", ROS2ScriptIntegrationModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2ScriptIntegrationModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2ScriptIntegrationModuleInterface, AZ::SystemAllocator);

    ROS2ScriptIntegrationModuleInterface::ROS2ScriptIntegrationModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            ROS2ScriptIntegrationSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2ScriptIntegrationModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2ScriptIntegrationSystemComponent>(),
        };
    }
} // namespace ROS2ScriptIntegration
