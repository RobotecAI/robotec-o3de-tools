
#include "TerrainShaperSystemComponent.h"

#include <TerrainShaper/TerrainShaperTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace TerrainShaper
{
    AZ_COMPONENT_IMPL(TerrainShaperSystemComponent, "TerrainShaperSystemComponent",
        TerrainShaperSystemComponentTypeId);

    void TerrainShaperSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TerrainShaperSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void TerrainShaperSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("TerrainShaperService"));
    }

    void TerrainShaperSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("TerrainShaperService"));
    }

    void TerrainShaperSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void TerrainShaperSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    TerrainShaperSystemComponent::TerrainShaperSystemComponent()
    {
        if (TerrainShaperInterface::Get() == nullptr)
        {
            TerrainShaperInterface::Register(this);
        }
    }

    TerrainShaperSystemComponent::~TerrainShaperSystemComponent()
    {
        if (TerrainShaperInterface::Get() == this)
        {
            TerrainShaperInterface::Unregister(this);
        }
    }

    void TerrainShaperSystemComponent::Init()
    {
    }

    void TerrainShaperSystemComponent::Activate()
    {
        TerrainShaperRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void TerrainShaperSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        TerrainShaperRequestBus::Handler::BusDisconnect();
    }

    void TerrainShaperSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace TerrainShaper
