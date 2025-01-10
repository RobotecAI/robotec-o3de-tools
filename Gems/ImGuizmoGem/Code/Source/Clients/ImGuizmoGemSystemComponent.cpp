
#include "ImGuizmoGemSystemComponent.h"

#include <ImGuizmoGem/ImGuizmoGemTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace ImGuizmoGem
{
    AZ_COMPONENT_IMPL(ImGuizmoGemSystemComponent, "ImGuizmoGemSystemComponent",
        ImGuizmoGemSystemComponentTypeId);

    void ImGuizmoGemSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImGuizmoGemSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void ImGuizmoGemSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ImGuizmoGemService"));
    }

    void ImGuizmoGemSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ImGuizmoGemService"));
    }

    void ImGuizmoGemSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ImGuizmoGemSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ImGuizmoGemSystemComponent::ImGuizmoGemSystemComponent()
    {
        if (ImGuizmoGemInterface::Get() == nullptr)
        {
            ImGuizmoGemInterface::Register(this);
        }
    }

    ImGuizmoGemSystemComponent::~ImGuizmoGemSystemComponent()
    {
        if (ImGuizmoGemInterface::Get() == this)
        {
            ImGuizmoGemInterface::Unregister(this);
        }
    }

    void ImGuizmoGemSystemComponent::Init()
    {
    }

    void ImGuizmoGemSystemComponent::Activate()
    {
        ImGuizmoGemRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ImGuizmoGemSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGuizmoGemRequestBus::Handler::BusDisconnect();
    }

    void ImGuizmoGemSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ImGuizmoGem
