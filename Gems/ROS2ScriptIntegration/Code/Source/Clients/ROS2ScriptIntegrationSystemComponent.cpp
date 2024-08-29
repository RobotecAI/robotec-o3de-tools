
#include "ROS2ScriptIntegrationSystemComponent.h"
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>
#include <ROS2ScriptIntegration/ROS2ScriptSubscriberBus.h>

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2ScriptIntegration
{
    AZ_COMPONENT_IMPL(
        ROS2ScriptIntegrationSystemComponent, "ROS2ScriptIntegrationSystemComponent", ROS2ScriptIntegrationSystemComponentTypeId);

    void ROS2ScriptIntegrationSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        SubscriberNotificationHandler::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ScriptIntegrationSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2ScriptIntegrationService"));
    }

    void ROS2ScriptIntegrationSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2ScriptIntegrationService"));
    }

    void ROS2ScriptIntegrationSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
        required.push_back(AZ_CRC_CE("ROS2ScriptIntegrationPublisherService"));
        required.push_back(AZ_CRC_CE("ROS2ScriptIntegrationSubscriberService"));
    }

    void ROS2ScriptIntegrationSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2ScriptIntegrationSystemComponent::ROS2ScriptIntegrationSystemComponent()
    {
        if (ROS2ScriptIntegrationInterface::Get() == nullptr)
        {
            ROS2ScriptIntegrationInterface::Register(this);
        }
    }

    ROS2ScriptIntegrationSystemComponent::~ROS2ScriptIntegrationSystemComponent()
    {
        if (ROS2ScriptIntegrationInterface::Get() == this)
        {
            ROS2ScriptIntegrationInterface::Unregister(this);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::Init()
    {
    }

    void ROS2ScriptIntegrationSystemComponent::Activate()
    {
        ROS2ScriptIntegrationRequestBus::Handler::BusConnect();
    }

    void ROS2ScriptIntegrationSystemComponent::Deactivate()
    {
        ROS2ScriptIntegrationRequestBus::Handler::BusDisconnect();
    }

} // namespace ROS2ScriptIntegration
