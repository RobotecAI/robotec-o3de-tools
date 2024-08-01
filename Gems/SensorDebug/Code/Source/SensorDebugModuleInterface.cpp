
#include "SensorDebugModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <SensorDebug/SensorDebugTypeIds.h>

#include <Clients/SensorDebugSystemComponent.h>

namespace SensorDebug
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(SensorDebugModuleInterface, "SensorDebugModuleInterface", SensorDebugModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(SensorDebugModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(SensorDebugModuleInterface, AZ::SystemAllocator);

    SensorDebugModuleInterface::SensorDebugModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                SensorDebugSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList SensorDebugModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<SensorDebugSystemComponent>(),
        };
    }
} // namespace SensorDebug
