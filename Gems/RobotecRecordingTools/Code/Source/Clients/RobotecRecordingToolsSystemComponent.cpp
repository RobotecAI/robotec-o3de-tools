
#include "RobotecRecordingToolsSystemComponent.h"

#include <RobotecRecordingTools/RobotecRecordingToolsTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecRecordingTools
{
    AZ_COMPONENT_IMPL(
        RobotecRecordingToolsSystemComponent, "RobotecRecordingToolsSystemComponent", RobotecRecordingToolsSystemComponentTypeId);

    void RobotecRecordingToolsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecRecordingToolsSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void RobotecRecordingToolsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecRecordingToolsService"));
    }

    void RobotecRecordingToolsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecRecordingToolsService"));
    }

    void RobotecRecordingToolsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotecRecordingToolsSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RobotecRecordingToolsSystemComponent::RobotecRecordingToolsSystemComponent()
    {
        if (RobotecRecordingToolsInterface::Get() == nullptr)
        {
            RobotecRecordingToolsInterface::Register(this);
        }
    }

    RobotecRecordingToolsSystemComponent::~RobotecRecordingToolsSystemComponent()
    {
        if (RobotecRecordingToolsInterface::Get() == this)
        {
            RobotecRecordingToolsInterface::Unregister(this);
        }
    }

    void RobotecRecordingToolsSystemComponent::Init()
    {
    }

    void RobotecRecordingToolsSystemComponent::Activate()
    {
        RobotecRecordingToolsRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void RobotecRecordingToolsSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        RobotecRecordingToolsRequestBus::Handler::BusDisconnect();
    }

    void RobotecRecordingToolsSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace RobotecRecordingTools
