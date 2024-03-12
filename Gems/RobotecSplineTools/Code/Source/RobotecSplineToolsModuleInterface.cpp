
#include "RobotecSplineToolsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <RobotecSplineTools/RobotecSplineToolsTypeIds.h>

#include <Clients/RobotecSplineToolsSystemComponent.h>

namespace RobotecSplineTools
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(RobotecSplineToolsModuleInterface,
        "RobotecSplineToolsModuleInterface", RobotecSplineToolsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(RobotecSplineToolsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(RobotecSplineToolsModuleInterface, AZ::SystemAllocator);

    RobotecSplineToolsModuleInterface::RobotecSplineToolsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            RobotecSplineToolsSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList RobotecSplineToolsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<RobotecSplineToolsSystemComponent>(),
        };
    }
} // namespace RobotecSplineTools
