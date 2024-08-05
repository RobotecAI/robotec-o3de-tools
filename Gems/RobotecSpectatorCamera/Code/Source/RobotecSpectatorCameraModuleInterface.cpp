
#include "RobotecSpectatorCameraModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

#include <Clients/RobotecSpectatorCameraSystemComponent.h>
#include <SpectatorCamera/SpectatorCameraComponent.h>

namespace RobotecSpectatorCamera
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        RobotecSpectatorCameraModuleInterface, "RobotecSpectatorCameraModuleInterface", RobotecSpectatorCameraModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(RobotecSpectatorCameraModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(RobotecSpectatorCameraModuleInterface, AZ::SystemAllocator);

    RobotecSpectatorCameraModuleInterface::RobotecSpectatorCameraModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                RobotecSpectatorCameraSystemComponent::CreateDescriptor(),
                SpectatorCameraComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList RobotecSpectatorCameraModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<RobotecSpectatorCameraSystemComponent>(),
        };
    }
} // namespace RobotecSpectatorCamera
