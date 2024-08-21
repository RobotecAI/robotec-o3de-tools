
#include "ExposeConsoleToRosEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ExposeConsoleToRos/ExposeConsoleToRosTypeIds.h>

namespace ExposeConsoleToRos
{
    AZ_COMPONENT_IMPL(
        ExposeConsoleToRosEditorSystemComponent,
        "ExposeConsoleToRosEditorSystemComponent",
        ExposeConsoleToRosEditorSystemComponentTypeId,
        BaseSystemComponent);

    void ExposeConsoleToRosEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ExposeConsoleToRosEditorSystemComponent, ExposeConsoleToRosSystemComponent>()->Version(0);
        }
    }

    ExposeConsoleToRosEditorSystemComponent::ExposeConsoleToRosEditorSystemComponent() = default;

    ExposeConsoleToRosEditorSystemComponent::~ExposeConsoleToRosEditorSystemComponent() = default;

    void ExposeConsoleToRosEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ExposeConsoleToRosEditorService"));
    }

    void ExposeConsoleToRosEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ExposeConsoleToRosEditorService"));
    }

    void ExposeConsoleToRosEditorSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ExposeConsoleToRosEditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ExposeConsoleToRosEditorSystemComponent::Activate()
    {
        ExposeConsoleToRosSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ExposeConsoleToRosEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ExposeConsoleToRosSystemComponent::Deactivate();
    }

} // namespace ExposeConsoleToRos
