
#include "SplineToolsEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SplineTools/SplineToolsTypeIds.h>

namespace SplineTools
{
    AZ_COMPONENT_IMPL(
        SplineToolsEditorSystemComponent, "SplineToolsEditorSystemComponent", SplineToolsEditorSystemComponentTypeId, BaseSystemComponent);

    void SplineToolsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineToolsEditorSystemComponent, SplineToolsSystemComponent>()->Version(0);
        }
    }

    void SplineToolsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SplineToolsEditorService"));
    }

    void SplineToolsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SplineToolsEditorService"));
    }

    void SplineToolsEditorSystemComponent::Activate()
    {
        SplineToolsSystemComponent::Activate();
    }

    void SplineToolsEditorSystemComponent::Deactivate()
    {
        SplineToolsSystemComponent::Deactivate();
    }

} // namespace SplineTools
