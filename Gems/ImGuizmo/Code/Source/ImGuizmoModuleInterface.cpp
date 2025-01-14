
#include "ImGuizmoModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ImGuizmo/ImGuizmoTypeIds.h>

#include <Clients/ImGuizmoSystemComponent.h>

namespace ImGuizmo
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ImGuizmoModuleInterface, "ImGuizmoModuleInterface", ImGuizmoModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ImGuizmoModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ImGuizmoModuleInterface, AZ::SystemAllocator);

    ImGuizmoModuleInterface::ImGuizmoModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ImGuizmoSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ImGuizmoModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ImGuizmoSystemComponent>(),
        };
    }
} // namespace ImGuizmo
