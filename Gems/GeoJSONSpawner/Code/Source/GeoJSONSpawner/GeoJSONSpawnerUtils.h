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

#include <AzCore/Math/Vector3.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace GeoJSONSpawner::GeoJSONUtils
{
    using Coordinates = AZStd::vector<AZStd::array<double, 3>>;
    using SpawnableCoordinatesMap = AZStd::unordered_map<AZStd::string, Coordinates>;
    using DespawnCallback = AZStd::function<void(AzFramework::EntitySpawnTicket::Id)>;
    using Ids = AZStd::unordered_set<int>;

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

    class GeometryObjectInfo
    {
    public:
        AZ_RTTI(GeometryObjectInfo, GeometryObjectInfoTypeId);
        static void Reflect(AZ::ReflectContext* context);
        GeometryObjectInfo() = default;
        virtual ~GeometryObjectInfo() = default;

        int m_id;
        AZStd::string m_name;
        Coordinates m_coordinates; // WGS84
    };

    class GeoJSONSpawnableEntityInfo
    {
    public:
        AZ_RTTI(GeoJSONSpawnableEntityInfo, GeoJSONSpawnableEntityInfoTypeId);
        static void Reflect(AZ::ReflectContext* context);
        GeoJSONSpawnableEntityInfo() = default;
        virtual ~GeoJSONSpawnableEntityInfo() = default;

        int m_id;
        AZStd::string m_name;
        AZStd::vector<AZ::Vector3> m_positions; // coordinates in level (X, Y, Z)
    };

    class GeoJSONSpawnableAssetConfiguration
    {
    public:
        AZ_RTTI(GeoJSONSpawnableAssetConfiguration, GeoJSONSpawnableAssetConfigurationTypeId);

        static void Reflect(AZ::ReflectContext* context);
        GeoJSONSpawnableAssetConfiguration() = default;
        virtual ~GeoJSONSpawnableAssetConfiguration() = default;

        AZStd::string m_name;
        AZ::Data::Asset<AzFramework::Spawnable> m_spawnable;
        AZ::Vector3 m_positionStdDev{ 0.0f };
        AZ::Vector3 m_rotationStdDev{ 0.0f };
        float m_scaleStdDev{ 0.1f };
        bool m_placeOnTerrain{ false };
        float m_raycastStartingHeight{ 0.0f };
    };

    AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> SpawnEntities(
        AZStd::vector<GeoJSONSpawnableEntityInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations,
        AZ::u64 defaultSeed,
        const AZStd::string& physicsSceneName = AZStd::string(),
        AZ::EntityId parentId = AZ::EntityId());

    void DespawnEntity(AzFramework::EntitySpawnTicket& ticket, DespawnCallback callback);

    AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration> GetSpawnableAssetFromVector(
        const AZStd::vector<GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations);

    AZStd::vector<GeoJSONSpawnableEntityInfo> GetSpawnableEntitiesFromGeometryObjectVector(
        const AZStd::vector<GeometryObjectInfo>& geometryObjects,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations);

    bool ValidateGeoJSON(const rapidjson::Document& geoJsonDocument);
    AZStd::vector<GeometryObjectInfo> ParseJSONFromFile(const AZStd::string& filePath);
    AZStd::vector<GeometryObjectInfo> ParseJSONFromRawString(const AZStd::string& rawGeoJson);
    Coordinates ExtractPoints(const rapidjson::Value& geometry);
    Ids ExtractIdsFromRawString(const AZStd::string& rawGeoJson);
    GeometryType GetGeometryType(const AZStd::string& geometryType);
} // namespace GeoJSONSpawner::GeoJSONUtils
