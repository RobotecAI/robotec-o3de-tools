
#include "RobotecSpectatorCameraSystemComponent.h"

#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecSpectatorCamera
{
    AZ_COMPONENT_IMPL(
        RobotecSpectatorCameraSystemComponent, "RobotecSpectatorCameraSystemComponent", RobotecSpectatorCameraSystemComponentTypeId);

    void RobotecSpectatorCameraSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecSpectatorCameraSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void RobotecSpectatorCameraSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecSpectatorCameraService"));
    }

    void RobotecSpectatorCameraSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecSpectatorCameraService"));
    }

    void RobotecSpectatorCameraSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotecSpectatorCameraSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RobotecSpectatorCameraSystemComponent::RobotecSpectatorCameraSystemComponent()
    {
    }

    RobotecSpectatorCameraSystemComponent::~RobotecSpectatorCameraSystemComponent()
    {
    }

    void RobotecSpectatorCameraSystemComponent::Init()
    {
    }

    void RobotecSpectatorCameraSystemComponent::Activate()
    {
    }

    void RobotecSpectatorCameraSystemComponent::Deactivate()
    {
    }
} // namespace RobotecSpectatorCamera
