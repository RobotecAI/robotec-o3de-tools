
#include <AzCore/Serialization/SerializeContext.h>
#include "DisableMainViewEditorSystemComponent.h"

#include <DisableMainView/DisableMainViewTypeIds.h>

namespace DisableMainView {
    AZ_COMPONENT_IMPL(DisableMainViewEditorSystemComponent, "DisableMainViewEditorSystemComponent",
                      DisableMainViewEditorSystemComponentTypeId, BaseSystemComponent);

    void DisableMainViewEditorSystemComponent::Reflect(AZ::ReflectContext *context) {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext *>(context)) {
            serializeContext->Class<DisableMainViewEditorSystemComponent, DisableMainViewSystemComponent>()
                    ->Version(0);
        }
    }

    DisableMainViewEditorSystemComponent::DisableMainViewEditorSystemComponent() = default;

    DisableMainViewEditorSystemComponent::~DisableMainViewEditorSystemComponent() = default;

    void
    DisableMainViewEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType &provided) {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("DisableMainViewEditorService"));
    }

    void DisableMainViewEditorSystemComponent::GetIncompatibleServices(
            AZ::ComponentDescriptor::DependencyArrayType &incompatible) {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("DisableMainViewEditorService"));
    }

    void DisableMainViewEditorSystemComponent::GetRequiredServices(
            [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType &required) {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void DisableMainViewEditorSystemComponent::GetDependentServices(
            [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType &dependent) {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void DisableMainViewEditorSystemComponent::Activate() {
        DisableMainViewSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void DisableMainViewEditorSystemComponent::Deactivate() {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        DisableMainViewSystemComponent::Deactivate();
    }

} // namespace DisableMainView
