
#include "RobotecImGuiModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <RobotecImGui/RobotecImGuiTypeIds.h>

#include <Clients/RobotecImGuiSystemComponent.h>
#include <ImGui/SensorImGui.h>

namespace RobotecImGui
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(RobotecImGuiModuleInterface,
        "RobotecImGuiModuleInterface", RobotecImGuiModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(RobotecImGuiModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(RobotecImGuiModuleInterface, AZ::SystemAllocator);

    RobotecImGuiModuleInterface::RobotecImGuiModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            RobotecImGuiSystemComponent::CreateDescriptor(),
            SensorImGui::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList RobotecImGuiModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<RobotecImGuiSystemComponent>(),
        };
    }
} // namespace RobotecImGui
