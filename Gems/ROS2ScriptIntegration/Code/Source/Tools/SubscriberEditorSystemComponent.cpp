
#include "SubscriberEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>

namespace ROS2ScriptIntegration
{
    AZ_COMPONENT_IMPL(
        SubscriberEditorSystemComponent, "SubscriberEditorSystemComponent", SubscriberEditorSystemComponentTypeId, BaseSystemComponent);

    void SubscriberEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SubscriberEditorSystemComponent, SubscriberSystemComponent>()->Version(0);
        }
    }

    SubscriberEditorSystemComponent::SubscriberEditorSystemComponent() = default;

    SubscriberEditorSystemComponent::~SubscriberEditorSystemComponent() = default;

    void SubscriberEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SubscriberEditorService"));
    }

    void SubscriberEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SubscriberEditorService"));
    }

    void SubscriberEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SubscriberEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void SubscriberEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        BaseSystemComponent::Activate();
    }

    void SubscriberEditorSystemComponent::Deactivate()
    {
        BaseSystemComponent::Deactivate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
    }

} // namespace ROS2ScriptIntegration
