
#include "SmoothingEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <Smoothing/SmoothingTypeIds.h>

namespace Smoothing
{
    AZ_COMPONENT_IMPL(
        SmoothingEditorSystemComponent, "SmoothingEditorSystemComponent", SmoothingEditorSystemComponentTypeId, BaseSystemComponent);

    void SmoothingEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SmoothingEditorSystemComponent, SmoothingSystemComponent>()->Version(0);
        }
    }

    SmoothingEditorSystemComponent::SmoothingEditorSystemComponent() = default;

    SmoothingEditorSystemComponent::~SmoothingEditorSystemComponent() = default;

    void SmoothingEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SmoothingEditorService"));
    }

    void SmoothingEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SmoothingEditorService"));
    }

    void SmoothingEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SmoothingEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void SmoothingEditorSystemComponent::Activate()
    {
        SmoothingSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void SmoothingEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        SmoothingSystemComponent::Deactivate();
    }

} // namespace Smoothing
