
#include "RobotecSplineToolsSystemComponent.h"

#include <RobotecSplineTools/RobotecSplineToolsTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecSplineTools
{
    AZ_COMPONENT_IMPL(RobotecSplineToolsSystemComponent, "RobotecSplineToolsSystemComponent",
        RobotecSplineToolsSystemComponentTypeId);

    void RobotecSplineToolsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecSplineToolsSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void RobotecSplineToolsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecSplineToolsService"));
    }

    void RobotecSplineToolsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecSplineToolsService"));
    }

    void RobotecSplineToolsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotecSplineToolsSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RobotecSplineToolsSystemComponent::RobotecSplineToolsSystemComponent()
    {
        if (RobotecSplineToolsInterface::Get() == nullptr)
        {
            RobotecSplineToolsInterface::Register(this);
        }
    }

    RobotecSplineToolsSystemComponent::~RobotecSplineToolsSystemComponent()
    {
        if (RobotecSplineToolsInterface::Get() == this)
        {
            RobotecSplineToolsInterface::Unregister(this);
        }
    }

    void RobotecSplineToolsSystemComponent::Init()
    {
    }

    void RobotecSplineToolsSystemComponent::Activate()
    {
        RobotecSplineToolsRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void RobotecSplineToolsSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        RobotecSplineToolsRequestBus::Handler::BusDisconnect();
    }

    void RobotecSplineToolsSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace RobotecSplineTools
