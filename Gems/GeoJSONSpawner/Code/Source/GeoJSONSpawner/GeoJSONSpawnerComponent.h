/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "GeoJSONSpawnerConfiguration.h"

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/Component.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

#include <GeoJSONSpawner/GeoJSONSpawnerBus.h>
#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

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

        AZ_COMPONENT(GeoJSONSpawnerComponent, GeoJSONSpawnerComponentTypeId);

        GeoJSONSpawnerComponent() = default;
        GeoJSONSpawnerComponent(const GeoJSONSpawnerConfiguration& configuration);
        ~GeoJSONSpawnerComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        // GeoJSONSpawnerRequestBus::Handler overrides...
        void Spawn(const AZStd::string& rawJsonString) override;

    private:
        bool ValidateGeoJSON(const rapidjson::Document& geoJsonDocument);
        SpawnableCoordinatesMap ParseGeoJSON(const AZStd::string& rawJsonString);
        Coordinates ExtractPoints(const rapidjson::Value& geometry);

        GeometryType GetGeometryType(const AZStd::string& geometryType);

        AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> m_spawnableTickets;
        GeoJSONSpawnerConfiguration m_configuration;
    };
} // namespace GeoJSONSpawner