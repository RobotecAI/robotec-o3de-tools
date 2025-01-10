
#include "ImGuizmoSystemComponent.h"

#include <ImGuizmo/ImGuizmoTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace ImGuizmo
{
    AZ_COMPONENT_IMPL(ImGuizmoSystemComponent, "ImGuizmoSystemComponent",
        ImGuizmoSystemComponentTypeId);

    void ImGuizmoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImGuizmoSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void ImGuizmoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ImGuizmoService"));
    }

    void ImGuizmoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ImGuizmoService"));
    }

    void ImGuizmoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ImGuizmoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ImGuizmoSystemComponent::ImGuizmoSystemComponent()
    {
        if (ImGuizmoInterface::Get() == nullptr)
        {
            ImGuizmoInterface::Register(this);
        }
    }

    ImGuizmoSystemComponent::~ImGuizmoSystemComponent()
    {
        if (ImGuizmoInterface::Get() == this)
        {
            ImGuizmoInterface::Unregister(this);
        }
    }

    void ImGuizmoSystemComponent::Init()
    {
    }

    void ImGuizmoSystemComponent::Activate()
    {
        ImGuizmoRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ImGuizmoSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGuizmoRequestBus::Handler::BusDisconnect();
    }

    void ImGuizmoSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ImGuizmo
