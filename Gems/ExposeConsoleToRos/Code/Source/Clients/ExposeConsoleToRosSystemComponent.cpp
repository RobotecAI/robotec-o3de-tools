
#include "ExposeConsoleToRosSystemComponent.h"

#include <ExposeConsoleToRos/ExposeConsoleToRosTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace ExposeConsoleToRos
{
    AZ_COMPONENT_IMPL(ExposeConsoleToRosSystemComponent, "ExposeConsoleToRosSystemComponent", ExposeConsoleToRosSystemComponentTypeId);

    void ExposeConsoleToRosSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ExposeConsoleToRosSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ExposeConsoleToRosSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ExposeConsoleToRosService"));
    }

    void ExposeConsoleToRosSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ExposeConsoleToRosService"));
    }

    void ExposeConsoleToRosSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ExposeConsoleToRosSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ExposeConsoleToRosSystemComponent::ExposeConsoleToRosSystemComponent()
    {
    }

    ExposeConsoleToRosSystemComponent::~ExposeConsoleToRosSystemComponent()
    {
    }

    void ExposeConsoleToRosSystemComponent::Init()
    {
    }

    void ExposeConsoleToRosSystemComponent::Activate()
    {
    }

    void ExposeConsoleToRosSystemComponent::Deactivate()
    {
    }

    void ExposeConsoleToRosSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ExposeConsoleToRos
