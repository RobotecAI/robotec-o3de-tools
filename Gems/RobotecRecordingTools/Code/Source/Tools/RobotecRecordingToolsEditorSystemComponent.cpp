
#include <AzCore/Serialization/SerializeContext.h>
#include "RobotecRecordingToolsEditorSystemComponent.h"

#include <RobotecRecordingTools/RobotecRecordingToolsTypeIds.h>

namespace RobotecRecordingTools
{
    AZ_COMPONENT_IMPL(RobotecRecordingToolsEditorSystemComponent, "RobotecRecordingToolsEditorSystemComponent",
        RobotecRecordingToolsEditorSystemComponentTypeId, BaseSystemComponent);

    void RobotecRecordingToolsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecRecordingToolsEditorSystemComponent, RobotecRecordingToolsSystemComponent>()
                ->Version(0);
        }
    }

    RobotecRecordingToolsEditorSystemComponent::RobotecRecordingToolsEditorSystemComponent() = default;

    RobotecRecordingToolsEditorSystemComponent::~RobotecRecordingToolsEditorSystemComponent() = default;

    void RobotecRecordingToolsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("RobotecRecordingToolsEditorService"));
    }

    void RobotecRecordingToolsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("RobotecRecordingToolsEditorService"));
    }

    void RobotecRecordingToolsEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void RobotecRecordingToolsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void RobotecRecordingToolsEditorSystemComponent::Activate()
    {
        RobotecRecordingToolsSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void RobotecRecordingToolsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        RobotecRecordingToolsSystemComponent::Deactivate();
    }

} // namespace RobotecRecordingTools
