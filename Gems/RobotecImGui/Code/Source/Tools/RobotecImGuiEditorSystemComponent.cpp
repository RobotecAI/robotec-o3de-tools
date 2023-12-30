
#include <AzCore/Serialization/SerializeContext.h>
#include "RobotecImGuiEditorSystemComponent.h"

#include <RobotecImGui/RobotecImGuiTypeIds.h>

namespace RobotecImGui
{
    AZ_COMPONENT_IMPL(RobotecImGuiEditorSystemComponent, "RobotecImGuiEditorSystemComponent",
        RobotecImGuiEditorSystemComponentTypeId, BaseSystemComponent);

    void RobotecImGuiEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecImGuiEditorSystemComponent, RobotecImGuiSystemComponent>()
                ->Version(0);
        }
    }

    RobotecImGuiEditorSystemComponent::RobotecImGuiEditorSystemComponent() = default;

    RobotecImGuiEditorSystemComponent::~RobotecImGuiEditorSystemComponent() = default;

    void RobotecImGuiEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("RobotecImGuiEditorService"));
    }

    void RobotecImGuiEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("RobotecImGuiEditorService"));
    }

    void RobotecImGuiEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void RobotecImGuiEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void RobotecImGuiEditorSystemComponent::Activate()
    {
        RobotecImGuiSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void RobotecImGuiEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        RobotecImGuiSystemComponent::Deactivate();
    }

} // namespace RobotecImGui
