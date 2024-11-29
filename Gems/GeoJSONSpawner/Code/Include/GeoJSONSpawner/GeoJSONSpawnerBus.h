
#pragma once

#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace GeoJSONSpawner
{
    class GeoJSONSpawnerRequests : public AZ::ComponentBus
    {
    public:
        using BusIdType = AZ::EntityId;

        virtual void Spawn(const AZStd::string& rawJsonString) = 0;
        virtual void Modify(const AZStd::string& rawJsonString) = 0;
        virtual void DeleteAll() = 0;
        virtual void DeleteById(const AZStd::string& rawJsonString) = 0;
        virtual AZStd::string GetIds() const = 0;
    };


    using GeoJSONSpawnerRequestBus = AZ::EBus<GeoJSONSpawnerRequests>;

} // namespace GeoJSONSpawner
