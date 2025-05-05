/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "GeoJSONSpawner/GeoJSONSpawnerTypeIds.h"

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace GeoJSONSpawner::GeoJSONUtils
{
    using Coordinates = AZStd::vector<AZStd::array<double, 3>>;
    using SpawnableCoordinatesMap = AZStd::unordered_map<AZStd::string, Coordinates>;
    using SpawnCompletionCallback = AZStd::function<void()>;
    using DespawnCallback = AZStd::function<void(AzFramework::EntitySpawnTicket::Id)>;
    using TicketToSpawnPair = AZStd::pair<AzFramework::EntitySpawnTicket, AzFramework::SpawnAllEntitiesOptionalArgs>;
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

    //! Information about a location of GeoJSON feature.
    //! This information is generated from the GeoJSON string with coordinates in the WGS84 system. This object is then converted to the
    //! @class GeoJSONSpawnableEntityInfo, which uses coordinates converted form the WGS84 system to the level coordinates using @class
    //! GeoreferenceRequestsBus.
    //! @param m_name is the name of the spawnable entity configuration and should be identical to name in the @class
    //! GeoJSONSpawnableAssetConfiguration
    class FeatureObjectInfo
    {
    public:
        AZ_RTTI(FeatureObjectInfo, FeatureObjectInfoTypeId);
        static void Reflect(AZ::ReflectContext* context);
        FeatureObjectInfo() = default;
        virtual ~FeatureObjectInfo() = default;

        int m_id; //!< Required ID for the spawned entities group that are connected with this ID. This parameter is needed to modify and
                  //!< Delete groups by id
        AZStd::string m_name; //!< Required name of the spawnable entity configuration
        Coordinates m_coordinates; //!< Vector containing coordinates (in WGS84 system) of the entities connected with m_id
    };

    //! Information about a location to spawn an entity.
    //! This information is generated from the @class FeatureObjectInfo object with level coordinates system.
    //! @param m_name is the name of the spawnable entity configuration and should be identical to name in the @class
    //! GeoJSONSpawnableAssetConfiguration
    class GeoJSONSpawnableEntityInfo
    {
    public:
        AZ_RTTI(GeoJSONSpawnableEntityInfo, GeoJSONSpawnableEntityInfoTypeId);
        static void Reflect(AZ::ReflectContext* context);
        GeoJSONSpawnableEntityInfo() = default;
        virtual ~GeoJSONSpawnableEntityInfo() = default;

        int m_id; //!< Required ID for the spwaned entities group that are connected with this ID. This parameter is needed to modify and
                  //!< Delete groups by id
        AZStd::string m_name; //!< Required name of the spawnable entity configuration
        AZStd::vector<AZ::Transform> m_positions; //!< Vector containing level coordinates of the entities connected with m_id
    };

    //! Configuration for a spawnable asset.
    //! This configuration is used to configure the spawnable asset before spawning it.
    //! The @param m_name is used to identify the spawnable asset configuration and should be identical to `spawnable_name` passed with
    //! GeoJSON.
    //! The class allows user to randomize parameters for position, rotation and scale.
    class GeoJSONSpawnableAssetConfiguration
    {
    public:
        AZ_RTTI(GeoJSONSpawnableAssetConfiguration, GeoJSONSpawnableAssetConfigurationTypeId);

        static void Reflect(AZ::ReflectContext* context);
        GeoJSONSpawnableAssetConfiguration() = default;
        virtual ~GeoJSONSpawnableAssetConfiguration() = default;

        AZStd::string m_name; //!< Name of the spawnable field
        AZ::Data::Asset<AzFramework::Spawnable> m_spawnable; //!< Spawnable asset
        AZ::Vector3 m_positionStdDev{ 0.0f }; //!< Standard deviation for position
        AZ::Vector3 m_rotationStdDev{ 0.0f }; //!< Standard deviation for rotation
        float m_scaleStdDev{ 0.1f }; //!< Standard deviation for scale
        bool m_placeOnTerrain{ false }; //!< Whether to raytrace to terrain and place the entity on the terrain
        float m_raytraceStartingHeight{ 0.0f }; //! Height at which raytrace will start (downwards) if the `m_placeOnTerrain` is set to
                                                //! true. If it is set to false, then this value is used as an altitude for spawned asset
    };

    //! This function creates and configures spawnable tickets. Tickets are only created for those `GeoJSONSpawnableEntityInfo` that can be
    //! spawned (uses spawnable that exists in the configuration and, if `Place on terrain` is set to `true`, can be snapped to the
    //! terrain). Because of this, the returned map may contain fewer prepared tickets than the number of points specified in the @param
    //! entitiesToSpawn
    //! @param entitiesToSpawn - vector of spawnable entity info
    //! @param spawnableAssetConfigurations - map of spawnable asset configuration
    //! @param defaultSeed - default seed value
    //! @param spawnCompletionCallback - callback function that will be called when ticket spawn is completed
    //! @param parentId - parent entity id to set for new entities
    //! @param physicsSceneName - physics scene name (AzPhysics::DefaultPhysicsSceneName or AzPhysics::EditorPhysicsSceneName)
    //! @return map containing group id and container of associated EntitySpawnTicket and SpawnAllEntitiesOptionalArgs pairs
    AZStd::unordered_map<int, AZStd::vector<TicketToSpawnPair>> PrepareTicketsToSpawn(
        AZStd::vector<GeoJSONSpawnableEntityInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations,
        AZ::u64 defaultSeed,
        SpawnCompletionCallback spawnCompletionCallback,
        const AZStd::string& physicsSceneName = AZStd::string(),
        AZ::EntityId parentId = AZ::EntityId());

    //! This function spawns entities using passed EntitySpawnTicket and SpawnAllEntitiesOptionalArgs pairs
    //! @param ticketsToSpawn - map containing group id and container of associated EntitySpawnTicket and SpawnAllEntitiesOptionalArgs pairs
    //! @return map containing group id with associated tickets
    AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> SpawnEntities(
        AZStd::unordered_map<int, AZStd::vector<TicketToSpawnPair>>& ticketsToSpawn);

    //! Despawn entities connected with given ticket. When ticked is despawned successfully then callback is called.
    //! @param ticket - ticket connected with spawned entities, which will be removed from the scene
    //! @param callback - function that is called when ticket is successfully despawned
    void DespawnEntity(AzFramework::EntitySpawnTicket& ticket, DespawnCallback callback);

    //! This function create map of spawnable asset configuration from vector of spawnable asset configuration, where
    //! the key is the name of the spawnable asset configuration
    //! @param spawnableAssetConfigurations - vector of spawnable asset configuration
    //! @return map of the spawnable asset configuration
    AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration> GetSpawnableAssetFromVector(
        const AZStd::vector<GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations);

    //! This function converts vector of the @class FeatureObjectInfo to the vector of the @class GeoJSONSpawnableEntityInfo
    //! @param featureObjects - vector of the @class FeatureObjectInfo which will be converted
    //! @param spawnableAssetConfigurations - map of the spawnable asset configuration used to get additional info for the converting
    //! process
    //! @return vector of the converted @class GeoJSONSpawnableEntityInfo
    AZStd::vector<GeoJSONSpawnableEntityInfo> GetSpawnableEntitiesFromFeatureObjectVector(
        const AZStd::vector<FeatureObjectInfo>& featureObjects,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations);

    //! This function validates passed rapidjson::Document with the GeoJSON schema
    //! @param geoJsonDocument - rapidjson::Document object with GeoJSON that should be validated against correctness with the schema
    //! @return boolean value indicating whether the passed json object is valid or not
    bool ValidateGeoJSON(const rapidjson::Document& geoJsonDocument);

    //! This function loads and parses json file from given path
    //! @param filePath - path to a json file
    //! @return vector of the @class FeatureObjectInfo read from the file
    AZStd::vector<FeatureObjectInfo> ParseJSONFromFile(const AZStd::string& filePath);

    //! This function parses json from raw string
    //! @param rawGeoJson - raw string containing GeoJSON
    //! @return vector of the @class FeatureObjectInfo read from the string
    AZStd::vector<FeatureObjectInfo> ParseJSONFromRawString(const AZStd::string& rawGeoJson);

    //! This function extracts raw points from `geometry` field, based on the given object type
    //! @param geometry - reference to the `geometry` field in the loaded rapidjson::Document GeoJSON
    //! @return vector of the coordinates in the WGS84 system
    Coordinates ExtractPoints(const rapidjson::Value& geometry);

    //! This function extracts value of the field `id` from the given raw json string
    //! @param rawGeoJson - raw string containing GeoJSON
    //! @return - unordered set of the extracted ids
    Ids ExtractIdsFromRawString(const AZStd::string& rawGeoJson);

    //! This function converts string geometry type to the @enum GeometryType
    //! @param geometryType - string with type of the geometry object
    //! @return enum connected with the given geometry type
    GeometryType GetGeometryType(const AZStd::string& geometryType);

    //! This function checks if the Terrain is available in the level.
    //! @returns True if level has any valid Terrain handlers, false otherwise.
    [[nodiscard]] bool IsTerrainAvailable();
} // namespace GeoJSONSpawner::GeoJSONUtils
