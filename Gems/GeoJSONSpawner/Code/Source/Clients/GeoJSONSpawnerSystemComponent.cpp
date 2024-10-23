
#include "GeoJSONSpawnerSystemComponent.h"

#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace GeoJSONSpawner
{
    AZ_COMPONENT_IMPL(GeoJSONSpawnerSystemComponent, "GeoJSONSpawnerSystemComponent",
        GeoJSONSpawnerSystemComponentTypeId);

    void GeoJSONSpawnerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void GeoJSONSpawnerSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GeoJSONSpawnerService"));
    }

    void GeoJSONSpawnerSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GeoJSONSpawnerService"));
    }

    void GeoJSONSpawnerSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void GeoJSONSpawnerSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    GeoJSONSpawnerSystemComponent::GeoJSONSpawnerSystemComponent()
    {
        if (GeoJSONSpawnerInterface::Get() == nullptr)
        {
            GeoJSONSpawnerInterface::Register(this);
        }
    }

    GeoJSONSpawnerSystemComponent::~GeoJSONSpawnerSystemComponent()
    {
        if (GeoJSONSpawnerInterface::Get() == this)
        {
            GeoJSONSpawnerInterface::Unregister(this);
        }
    }

    void GeoJSONSpawnerSystemComponent::Init()
    {
    }

    void GeoJSONSpawnerSystemComponent::Activate()
    {
        GeoJSONSpawnerRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void GeoJSONSpawnerSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        GeoJSONSpawnerRequestBus::Handler::BusDisconnect();
    }

    void GeoJSONSpawnerSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace GeoJSONSpawner
