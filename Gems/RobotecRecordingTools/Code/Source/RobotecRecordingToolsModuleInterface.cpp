
#include "RobotecRecordingToolsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <RobotecRecordingTools/RobotecRecordingToolsTypeIds.h>

#include <Cameras/CameraJoystick.h>
#include <Cameras/SplineCameraAnimation.h>
#include <Clients/RobotecRecordingToolsSystemComponent.h>

namespace RobotecRecordingTools
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        RobotecRecordingToolsModuleInterface, "RobotecRecordingToolsModuleInterface", RobotecRecordingToolsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(RobotecRecordingToolsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(RobotecRecordingToolsModuleInterface, AZ::SystemAllocator);

    RobotecRecordingToolsModuleInterface::RobotecRecordingToolsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                RobotecRecordingToolsSystemComponent::CreateDescriptor(),
                ROS2::Demo::SplineCameraAnimation::CreateDescriptor(),
                ROS2::Demo::CameraJoystick::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList RobotecRecordingToolsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<RobotecRecordingToolsSystemComponent>(),
        };
    }
} // namespace RobotecRecordingTools
