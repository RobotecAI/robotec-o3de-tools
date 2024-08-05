
#include "SmoothingSystemComponent.h"

#include <Smoothing/SmoothingTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace Smoothing
{
    AZ_COMPONENT_IMPL(SmoothingSystemComponent, "SmoothingSystemComponent", SmoothingSystemComponentTypeId);

    void SmoothingSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SmoothingSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void SmoothingSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SmoothingService"));
    }

    void SmoothingSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SmoothingService"));
    }

    void SmoothingSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void SmoothingSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void SmoothingSystemComponent::Init()
    {
    }

    void SmoothingSystemComponent::Activate()
    {
    }

    void SmoothingSystemComponent::Deactivate()
    {
    }

} // namespace Smoothing
