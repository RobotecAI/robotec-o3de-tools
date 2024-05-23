
#include "ROS2PoseControlSystemComponent.h"

#include <ROS2PoseControl/ROS2PoseControlTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2PoseControl/ROS2PoseControlConfiguration.h>

namespace ROS2PoseControl
{
    AZ_COMPONENT_IMPL(ROS2PoseControlSystemComponent, "ROS2PoseControlSystemComponent",
        ROS2PoseControlSystemComponentTypeId);

    void ROS2PoseControlSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2PoseControlConfiguration::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2PoseControlSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void ROS2PoseControlSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2PoseControlService"));
    }

    void ROS2PoseControlSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2PoseControlService"));
    }

    void ROS2PoseControlSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2PoseControlSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2PoseControlSystemComponent::ROS2PoseControlSystemComponent()
    {
    }

    ROS2PoseControlSystemComponent::~ROS2PoseControlSystemComponent()
    {
    }

    void ROS2PoseControlSystemComponent::Init()
    {
    }

    void ROS2PoseControlSystemComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2PoseControlSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ROS2PoseControlSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ROS2PoseControl
