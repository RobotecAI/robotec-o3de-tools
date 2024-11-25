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

#include "GeoJSONSpawner/GeoJSONSpawnerTypeIds.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace GeoJSONSpawner::GeoJSONUtils
{
    using Coordinates = AZStd::vector<AZStd::array<double, 2>>;
    using SpawnableCoordinatesMap = AZStd::unordered_map<AZStd::string, Coordinates>;

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

    struct GeometryObjectInfo
    {
        int m_id;
        AZStd::string m_name;
        Coordinates m_coordinates;
    };

    class GeoJSONSpawnerConfiguration
    {
    public:
        AZ_TYPE_INFO(GeoJSONSpawnerConfiguration, GeoJSONSpawnerConfigurationTypeId);

        static void Reflect(AZ::ReflectContext* context);

        AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> m_spawnableAssets;
        double m_altitude{ 0.0f };
        AZ::Data::AssetId m_geoJsonAssetId;
    };

    AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> SpawnEntities(
        const AZStd::vector<GeometryObjectInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>>& spawnableAssetConfiguration,
        AZ::EntityId parentId);

    bool ValidateGeoJSON(const rapidjson::Document& geoJsonDocument);
    AZStd::vector<GeometryObjectInfo> ParseJSONFromFile(const AZStd::string& filePath);
    AZStd::vector<GeometryObjectInfo> ParseJSONFromRawString(const AZStd::string& rawGeoJson);
    AZStd::vector<GeometryObjectInfo> ParseGeoJSON(const rapidjson::Document& geoJsonDocument);
    Coordinates ExtractPoints(const rapidjson::Value& geometry);
    GeometryType GetGeometryType(const AZStd::string& geometryType);
} // namespace GeoJSONSpawner::GeoJSONUtils
