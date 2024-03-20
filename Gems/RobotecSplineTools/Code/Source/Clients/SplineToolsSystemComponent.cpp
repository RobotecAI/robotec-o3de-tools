
#include "SplineToolsSystemComponent.h"

#include <SplineTools/SplineToolsTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace SplineTools
{
    AZ_COMPONENT_IMPL(SplineToolsSystemComponent, "SplineToolsSystemComponent", SplineToolsSystemComponentTypeId);

    void SplineToolsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineToolsSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void SplineToolsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SplineToolsService"));
    }

    void SplineToolsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SplineToolsService"));
    }

    void SplineToolsSystemComponent::Activate()
    {
    }

    void SplineToolsSystemComponent::Deactivate()
    {
    }

} // namespace SplineTools
