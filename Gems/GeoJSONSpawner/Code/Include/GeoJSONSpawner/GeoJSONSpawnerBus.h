
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
    };

    class GeoJSONSpawnerBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using GeoJSONSpawnerRequestBus = AZ::EBus<GeoJSONSpawnerRequests, GeoJSONSpawnerBusTraits>;
    using GeoJSONSpawnerInterface = AZ::Interface<GeoJSONSpawnerRequests>;

} // namespace GeoJSONSpawner
