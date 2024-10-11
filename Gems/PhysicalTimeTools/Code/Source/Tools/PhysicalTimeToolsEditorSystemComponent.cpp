
#include <AzCore/Serialization/SerializeContext.h>
#include "PhysicalTimeToolsEditorSystemComponent.h"

#include <PhysicalTimeTools/PhysicalTimeToolsTypeIds.h>

namespace PhysicalTimeTools
{
    AZ_COMPONENT_IMPL(PhysicalTimeToolsEditorSystemComponent, "PhysicalTimeToolsEditorSystemComponent",
        PhysicalTimeToolsEditorSystemComponentTypeId, BaseSystemComponent);

    void PhysicalTimeToolsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PhysicalTimeToolsEditorSystemComponent, PhysicalTimeToolsSystemComponent>()
                ->Version(0);
        }
    }

    PhysicalTimeToolsEditorSystemComponent::PhysicalTimeToolsEditorSystemComponent() = default;

    PhysicalTimeToolsEditorSystemComponent::~PhysicalTimeToolsEditorSystemComponent() = default;

    void PhysicalTimeToolsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("PhysicalTimeToolsEditorService"));
    }

    void PhysicalTimeToolsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("PhysicalTimeToolsEditorService"));
    }

    void PhysicalTimeToolsEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void PhysicalTimeToolsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void PhysicalTimeToolsEditorSystemComponent::Activate()
    {
        PhysicalTimeToolsSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void PhysicalTimeToolsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        PhysicalTimeToolsSystemComponent::Deactivate();
    }

} // namespace PhysicalTimeTools
