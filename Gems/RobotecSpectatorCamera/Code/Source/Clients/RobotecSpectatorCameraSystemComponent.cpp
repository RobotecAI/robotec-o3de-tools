
#include "RobotecSpectatorCameraSystemComponent.h"

#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecSpectatorCamera
{
    AZ_COMPONENT_IMPL(RobotecSpectatorCameraSystemComponent, "RobotecSpectatorCameraSystemComponent",
        RobotecSpectatorCameraSystemComponentTypeId);

    void RobotecSpectatorCameraSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecSpectatorCameraSystemComponent, AZ::Component>()
                ->Version(0)
                ;
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

    void RobotecSpectatorCameraSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RobotecSpectatorCameraSystemComponent::RobotecSpectatorCameraSystemComponent()
    {
        if (RobotecSpectatorCameraInterface::Get() == nullptr)
        {
            RobotecSpectatorCameraInterface::Register(this);
        }
    }

    RobotecSpectatorCameraSystemComponent::~RobotecSpectatorCameraSystemComponent()
    {
        if (RobotecSpectatorCameraInterface::Get() == this)
        {
            RobotecSpectatorCameraInterface::Unregister(this);
        }
    }

    void RobotecSpectatorCameraSystemComponent::Init()
    {
    }

    void RobotecSpectatorCameraSystemComponent::Activate()
    {
        RobotecSpectatorCameraRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void RobotecSpectatorCameraSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        RobotecSpectatorCameraRequestBus::Handler::BusDisconnect();
    }

    void RobotecSpectatorCameraSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace RobotecSpectatorCamera
