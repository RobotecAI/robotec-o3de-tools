
#include "ImGuiProviderModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ImGuiProvider/ImGuiProviderTypeIds.h>

#include <Clients/ImGuiProviderSystemComponent.h>

namespace ImGuiProvider
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ImGuiProviderModuleInterface, "ImGuiProviderModuleInterface", ImGuiProviderModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ImGuiProviderModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ImGuiProviderModuleInterface, AZ::SystemAllocator);

    ImGuiProviderModuleInterface::ImGuiProviderModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the
        // the SerializeContext, BehaviorContext and EditContext. This happens through
        // the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ImGuiProviderSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ImGuiProviderModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ImGuiProviderSystemComponent>(),
        };
    }
} // namespace ImGuiProvider
