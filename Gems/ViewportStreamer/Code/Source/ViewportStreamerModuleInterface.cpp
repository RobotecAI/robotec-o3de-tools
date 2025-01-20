
#include "ViewportStreamerModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ViewportStreamer/ViewportStreamerTypeIds.h>

#include <Clients/ViewportStreamerSystemComponent.h>

namespace ViewportStreamer
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ViewportStreamerModuleInterface, "ViewportStreamerModuleInterface", ViewportStreamerModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ViewportStreamerModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ViewportStreamerModuleInterface, AZ::SystemAllocator);

    ViewportStreamerModuleInterface::ViewportStreamerModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ViewportStreamerSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ViewportStreamerModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ViewportStreamerSystemComponent>(),
        };
    }
} // namespace ViewportStreamer
