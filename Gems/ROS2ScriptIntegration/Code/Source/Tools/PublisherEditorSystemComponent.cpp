
#include "PublisherEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>

namespace ROS2ScriptIntegration
{
    AZ_COMPONENT_IMPL(
        PublisherEditorSystemComponent, "PublisherEditorSystemComponent", PublisherEditorSystemComponentTypeId, BaseSystemComponent);

    void PublisherEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PublisherEditorSystemComponent, PublisherSystemComponent>()->Version(0);
        }
    }

    PublisherEditorSystemComponent::PublisherEditorSystemComponent() = default;

    PublisherEditorSystemComponent::~PublisherEditorSystemComponent() = default;

    void PublisherEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("PublisherEditorService"));
    }

    void PublisherEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("PublisherEditorService"));
    }

    void PublisherEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void PublisherEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void PublisherEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        BaseSystemComponent::Activate();
    }

    void PublisherEditorSystemComponent::Deactivate()
    {
        BaseSystemComponent::Deactivate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
    }

} // namespace ROS2ScriptIntegration
