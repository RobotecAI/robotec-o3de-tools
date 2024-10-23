
#pragma once

#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace GeoJSONSpawner
{
    class GeoJSONSpawnerRequests
    {
    public:
        AZ_RTTI(GeoJSONSpawnerRequests, GeoJSONSpawnerRequestsTypeId);
        virtual ~GeoJSONSpawnerRequests() = default;
        // Put your public methods here

        virtual void Spawn(const AZStd::string& rawJsonString) = 0;
    };

    class GeoJSONSpawnerBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;
        //////////////////////////////////////////////////////////////////////////
    };

    using GeoJSONSpawnerRequestBus = AZ::EBus<GeoJSONSpawnerRequests, GeoJSONSpawnerBusTraits>;
    using GeoJSONSpawnerInterface = AZ::Interface<GeoJSONSpawnerRequests>;

} // namespace GeoJSONSpawner
