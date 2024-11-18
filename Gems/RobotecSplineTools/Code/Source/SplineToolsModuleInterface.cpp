
#include "SplineToolsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <SplineTools/SplineToolsTypeIds.h>

#include <Clients/SplineSubscriber.h>
#include <Clients/SplineToolsSystemComponent.h>
#include <Clients/VisualizeSplineComponent.h>

namespace SplineTools
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(SplineToolsModuleInterface, "SplineToolsModuleInterface", SplineToolsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(SplineToolsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(SplineToolsModuleInterface, AZ::SystemAllocator);

    SplineToolsModuleInterface::SplineToolsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            { SplineToolsSystemComponent::CreateDescriptor(),
              VisualizeSplineComponent::CreateDescriptor(),
              SplineSubscriber::CreateDescriptor() });
    }

    AZ::ComponentTypeList SplineToolsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<SplineToolsSystemComponent>(),
        };
    }
} // namespace SplineTools
