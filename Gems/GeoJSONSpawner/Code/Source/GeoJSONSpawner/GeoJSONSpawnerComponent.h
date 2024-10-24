#pragma once

#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

#include <AzCore/Asset/AssetCommon.h>

#include <AzFramework/Spawnable/Spawnable.h>

#include <AzCore/Component/Component.h>
#include <GeoJSONSpawner/GeoJSONSpawnerBus.h>

namespace GeoJSONSpawner
{
    enum class GeometryType
    {
        Point = 0,
        MultiPoint,
        LineString,
        MultiLineString,
        Polygon,
        MultiPolygon,
        GeometryCollection,
        Unknown
    };

    class GeoJSONSpawnerComponent
        : public AZ::Component
        , public GeoJSONSpawnerRequestBus::Handler
    {
    public:
        using Coordinates = AZStd::vector<AZStd::array<double, 2>>;
        using SpawnableCoordinatesMap = AZStd::unordered_map<AZStd::string, Coordinates>;

        AZ_COMPONENT(GeoJSONSpawnerComponent, "{839ede69-92f1-45b0-a60e-035d0b84e1fd}");

        GeoJSONSpawnerComponent() = default;
        ~GeoJSONSpawnerComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        // GeoJSONSpawnerRequestBus::Handler overrides...
        void Spawn(const AZStd::string& rawJsonString) override;

    private:
        AZStd::string FindAssetPath(const AZStd::string& assetName) const;
        AZStd::string LoadJSONSchema() const;
        bool ValidateGeoJSON(const rapidjson::Document& geoJsonDocument);
        SpawnableCoordinatesMap ParseGeoJSON(const AZStd::string& rawJsonString);
        Coordinates ExtractPoints(const rapidjson::Value& geometry);

        GeometryType GetGeometryType(const AZStd::string& geometryType);

        AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> m_spawnableAssets;
        AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> m_spawnableTickets;
        double m_longitude{ 0.0f };
    };
} // namespace GeoJSONSpawner