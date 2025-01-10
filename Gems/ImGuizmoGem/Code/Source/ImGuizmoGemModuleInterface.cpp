
#include "ImGuizmoGemModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ImGuizmoGem/ImGuizmoGemTypeIds.h>

#include <Clients/ImGuizmoGemSystemComponent.h>

namespace ImGuizmoGem
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ImGuizmoGemModuleInterface,
        "ImGuizmoGemModuleInterface", ImGuizmoGemModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ImGuizmoGemModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ImGuizmoGemModuleInterface, AZ::SystemAllocator);

    ImGuizmoGemModuleInterface::ImGuizmoGemModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            ImGuizmoGemSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ImGuizmoGemModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ImGuizmoGemSystemComponent>(),
        };
    }
} // namespace ImGuizmoGem
