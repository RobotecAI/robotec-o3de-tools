
#include <AzCore/Serialization/SerializeContext.h>
#include "RobotecSplineToolsEditorSystemComponent.h"

#include <RobotecSplineTools/RobotecSplineToolsTypeIds.h>

namespace RobotecSplineTools
{
    AZ_COMPONENT_IMPL(RobotecSplineToolsEditorSystemComponent, "RobotecSplineToolsEditorSystemComponent",
        RobotecSplineToolsEditorSystemComponentTypeId, BaseSystemComponent);

    void RobotecSplineToolsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecSplineToolsEditorSystemComponent, RobotecSplineToolsSystemComponent>()
                ->Version(0);
        }
    }

    RobotecSplineToolsEditorSystemComponent::RobotecSplineToolsEditorSystemComponent() = default;

    RobotecSplineToolsEditorSystemComponent::~RobotecSplineToolsEditorSystemComponent() = default;

    void RobotecSplineToolsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("RobotecSplineToolsEditorService"));
    }

    void RobotecSplineToolsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("RobotecSplineToolsEditorService"));
    }

    void RobotecSplineToolsEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void RobotecSplineToolsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void RobotecSplineToolsEditorSystemComponent::Activate()
    {
        RobotecSplineToolsSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void RobotecSplineToolsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        RobotecSplineToolsSystemComponent::Deactivate();
    }

} // namespace RobotecSplineTools
