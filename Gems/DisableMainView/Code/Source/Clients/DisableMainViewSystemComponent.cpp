
#include "DisableMainViewSystemComponent.h"

#include <DisableMainView/DisableMainViewTypeIds.h>

#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/API/ApplicationAPI.h>

namespace DisableMainView
{
    AZ_COMPONENT_IMPL(DisableMainViewSystemComponent, "DisableMainViewSystemComponent", DisableMainViewSystemComponentTypeId);

    AZ_CVAR(bool, cl_disableMainView, false, nullptr, AZ::ConsoleFunctorFlags::Null, "Disable the main view");

    void DisableMainViewSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<DisableMainViewSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void DisableMainViewSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("DisableMainViewService"));
    }

    void DisableMainViewSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("DisableMainViewService"));
    }

    void DisableMainViewSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void DisableMainViewSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void DisableMainViewSystemComponent::Init()
    {
    }

    void DisableMainViewSystemComponent::Activate()
    {
        if (cl_disableMainView)
        {
            AZ::SystemTickBus::Handler::BusConnect();
        }
    }

    void DisableMainViewSystemComponent::Deactivate()
    {
        if (AZ::SystemTickBus::Handler::BusIsConnected())
        {
            AZ::SystemTickBus::Handler::BusDisconnect();
        }
    }

    void DisableMainViewSystemComponent::OnSystemTick()
    {
        if (cl_disableMainView)
        {
            auto viewportContext = AZ::RPI::ViewportContextRequests::Get()->GetDefaultViewportContext();
            viewportContext->SetCameraTransform(AZ::Transform::CreateIdentity());
            viewportContext->SetCameraProjectionMatrix(AZ::Matrix4x4::CreateZero());

        }
    }

} // namespace DisableMainView
