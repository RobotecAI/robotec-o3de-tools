
#include "DisableMainViewSystemComponent.h"

#include <DisableMainView/DisableMainViewTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <AzCore/Console/IConsole.h>
#include <AzFramework/API/ApplicationAPI.h>

namespace DisableMainView {
    AZ_COMPONENT_IMPL(DisableMainViewSystemComponent, "DisableMainViewSystemComponent",
                      DisableMainViewSystemComponentTypeId);

    AZ_CVAR(bool, cl_disableMainView, false, nullptr, AZ::ConsoleFunctorFlags::Null, "Disable the main view");

    void DisableMainViewSystemComponent::Reflect(AZ::ReflectContext *context) {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext *>(context)) {
            serializeContext->Class<DisableMainViewSystemComponent, AZ::Component>()
                    ->Version(0);
        }
    }

    void DisableMainViewSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType &provided) {
        provided.push_back(AZ_CRC_CE("DisableMainViewService"));
    }

    void DisableMainViewSystemComponent::GetIncompatibleServices(
            AZ::ComponentDescriptor::DependencyArrayType &incompatible) {
        incompatible.push_back(AZ_CRC_CE("DisableMainViewService"));
    }

    void DisableMainViewSystemComponent::GetRequiredServices(
            [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType &required) {
    }

    void DisableMainViewSystemComponent::GetDependentServices(
            [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType &dependent) {
    }


    void DisableMainViewSystemComponent::Init() {
    }

    void DisableMainViewSystemComponent::Activate() {

        if (cl_disableMainView) {
            AZ::ApplicationTypeQuery appType;
            AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
            AZ_Warning("DisableMainView", appType.IsGame(),
                       "DisableMainViewSystemComponent is only supported in game mode");
            if (appType.IsGame()) {
                AZ::SystemTickBus::QueueFunction([]() {
                    auto viewportContext = AZ::RPI::ViewportContextRequests::Get()->GetDefaultViewportContext();
                    AZ_Assert(viewportContext, "No default viewport context found");
                    viewportContext->GetWindowContext()->Shutdown();

                });
            }
        }
    }

    void DisableMainViewSystemComponent::Deactivate() {

    }

} // namespace DisableMainView
