
#include "RobotecImGuiSystemComponent.h"

#include <RobotecImGui/RobotecImGuiTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecImGui
{
    AZ_COMPONENT_IMPL(RobotecImGuiSystemComponent, "RobotecImGuiSystemComponent",
        RobotecImGuiSystemComponentTypeId);

    void RobotecImGuiSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecImGuiSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void RobotecImGuiSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecImGuiService"));
    }

    void RobotecImGuiSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecImGuiService"));
    }

    void RobotecImGuiSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotecImGuiSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RobotecImGuiSystemComponent::RobotecImGuiSystemComponent()
    {
        if (RobotecImGuiInterface::Get() == nullptr)
        {
            RobotecImGuiInterface::Register(this);
        }
    }

    RobotecImGuiSystemComponent::~RobotecImGuiSystemComponent()
    {
        if (RobotecImGuiInterface::Get() == this)
        {
            RobotecImGuiInterface::Unregister(this);
        }
    }

    void RobotecImGuiSystemComponent::Init()
    {
    }

    void RobotecImGuiSystemComponent::Activate()
    {
        RobotecImGuiRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void RobotecImGuiSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        RobotecImGuiRequestBus::Handler::BusDisconnect();
    }

    void RobotecImGuiSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace RobotecImGui
