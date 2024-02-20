
#include <AzCore/Serialization/SerializeContext.h>
#include "ROS2ScriptIntegrationEditorSystemComponent.h"

#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>

namespace ROS2ScriptIntegration
{
    AZ_COMPONENT_IMPL(ROS2ScriptIntegrationEditorSystemComponent, "ROS2ScriptIntegrationEditorSystemComponent",
        ROS2ScriptIntegrationEditorSystemComponentTypeId, BaseSystemComponent);

    void ROS2ScriptIntegrationEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ScriptIntegrationEditorSystemComponent, ROS2ScriptIntegrationSystemComponent>()
                ->Version(0);
        }
    }

    ROS2ScriptIntegrationEditorSystemComponent::ROS2ScriptIntegrationEditorSystemComponent() = default;

    ROS2ScriptIntegrationEditorSystemComponent::~ROS2ScriptIntegrationEditorSystemComponent() = default;

    void ROS2ScriptIntegrationEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2ScriptIntegrationEditorService"));
    }

    void ROS2ScriptIntegrationEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2ScriptIntegrationEditorService"));
    }

    void ROS2ScriptIntegrationEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ROS2ScriptIntegrationEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ROS2ScriptIntegrationEditorSystemComponent::Activate()
    {
        ROS2ScriptIntegrationSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ROS2ScriptIntegrationEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ROS2ScriptIntegrationSystemComponent::Deactivate();
    }

} // namespace ROS2ScriptIntegration
