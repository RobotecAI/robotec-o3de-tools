
#include <AzCore/Serialization/SerializeContext.h>
#include "SplineToolsEditorSystemComponent.h"

#include <SplineTools/SplineToolsTypeIds.h>

namespace SplineTools
{
    AZ_COMPONENT_IMPL(SplineToolsEditorSystemComponent, "SplineToolsEditorSystemComponent",
        SplineToolsEditorSystemComponentTypeId, BaseSystemComponent);

    void SplineToolsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineToolsEditorSystemComponent, SplineToolsSystemComponent>()
                ->Version(0);
        }
    }

    SplineToolsEditorSystemComponent::SplineToolsEditorSystemComponent() = default;

    SplineToolsEditorSystemComponent::~SplineToolsEditorSystemComponent() = default;

    void SplineToolsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SplineToolsEditorService"));
    }

    void SplineToolsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SplineToolsEditorService"));
    }

    void SplineToolsEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SplineToolsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void SplineToolsEditorSystemComponent::Activate()
    {
        SplineToolsSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void SplineToolsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        SplineToolsSystemComponent::Deactivate();
    }

} // namespace SplineTools
