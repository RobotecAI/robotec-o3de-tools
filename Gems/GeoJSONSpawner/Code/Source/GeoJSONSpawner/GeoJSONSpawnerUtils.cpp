/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerUtils.h"
#include "Schemas/GeoJSONSchema.h"

#include <AzFramework/Terrain/TerrainDataRequestBus.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/Georeference/GeoreferenceBus.h>
#include <random>
#include <rapidjson/schema.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>

namespace GeoJSONSpawner::GeoJSONUtils
{
    void FeatureObjectInfo::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FeatureObjectInfo>()
                ->Version(0)
                ->Field("Id", &FeatureObjectInfo::m_id)
                ->Field("Name", &FeatureObjectInfo::m_name)
                ->Field("Coordinates", &FeatureObjectInfo::m_coordinates);
        }
    }

    void GeoJSONSpawnableEntityInfo::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnableEntityInfo>()
                ->Version(0)
                ->Field("Id", &GeoJSONSpawnableEntityInfo::m_id)
                ->Field("Name", &GeoJSONSpawnableEntityInfo::m_name)
                ->Field("Positions", &GeoJSONSpawnableEntityInfo::m_positions);
        }
    }

    void GeoJSONSpawnableAssetConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnableAssetConfiguration>()
                ->Version(0)
                ->Field("Name", &GeoJSONSpawnableAssetConfiguration::m_name)
                ->Field("Spawnable", &GeoJSONSpawnableAssetConfiguration::m_spawnable)
                ->Field("PositionStdDev", &GeoJSONSpawnableAssetConfiguration::m_positionStdDev)
                ->Field("RotationStdDev", &GeoJSONSpawnableAssetConfiguration::m_rotationStdDev)
                ->Field("ScaleStdDev", &GeoJSONSpawnableAssetConfiguration::m_scaleStdDev)
                ->Field("PlaceOnTerrain", &GeoJSONSpawnableAssetConfiguration::m_placeOnTerrain)
                ->Field("RaycastStartingHeight", &GeoJSONSpawnableAssetConfiguration::m_raytraceStartingHeight);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GeoJSONSpawnableAssetConfiguration>("GeoJSONSpawnerConfiguration", "GeoJSON Spawner Configuration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSONSpawnerConfiguration")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &GeoJSONSpawnableAssetConfiguration::m_name, "Name", "Name of the spawnable")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GeoJSONSpawnableAssetConfiguration::m_spawnable, "Spawnable", "Spawnable")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_positionStdDev,
                        "Position std. dev",
                        "Position standard deviation, in meters.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_rotationStdDev,
                        "Rotation std. dev",
                        "Rotation standard deviation, in degrees.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_scaleStdDev,
                        "Scale std. dev",
                        "Scale standard deviation.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_placeOnTerrain,
                        "Place on terrain",
                        "Performscene query raytrace to place spawnable on terrain.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_raytraceStartingHeight,
                        "Raytrace starting height",
                        "Height at which raytrace will start downwards. If Place on terrain option is set to false, this value is used as "
                        "an altitude coordinate.");
            }
        }
    }

    AZStd::optional<AZ::Vector3> RaytraceTerrain(
        const AZ::Vector3& location, const AzPhysics::SceneHandle sceneHandle, const AZ::Vector3& gravityDirection, float maxDistance)
    {
        AZStd::optional<AZ::Vector3> hitPosition = AZStd::nullopt;

        if (sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return hitPosition;
        }

        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Unable to get pointer to physics system interface.");
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Unable to get pointer to scene interface.");

        if (!sceneInterface || !physicsSystem)
        {
            return hitPosition;
        }

        AzPhysics::RayCastRequest request;
        request.m_start = location;
        request.m_direction = gravityDirection;
        request.m_distance = maxDistance;

        AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(sceneHandle, &request);

        if (!result.m_hits.empty())
        {
            hitPosition = result.m_hits.front().m_position;
        }

        return hitPosition;
    }

    AZ::Transform GetRandomTransform(
        const AZ::Vector3& stdDevTranslation, const AZ::Vector3& stdDevRotation, float stdDevScale, std::mt19937& gen)
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZStd::vector<std::normal_distribution<float>> distributions;
        distributions.emplace_back(0.0f, stdDevTranslation.GetX());
        distributions.emplace_back(0.0f, stdDevTranslation.GetY());
        distributions.emplace_back(0.0f, stdDevTranslation.GetZ());
        distributions.emplace_back(0.0f, stdDevRotation.GetX());
        distributions.emplace_back(0.0f, stdDevRotation.GetY());
        distributions.emplace_back(0.0f, stdDevRotation.GetZ());
        distributions.emplace_back(1.0f, stdDevScale);

        transform.SetTranslation(AZ::Vector3(distributions[0](gen), distributions[1](gen), distributions[2](gen)));
        const AZ::Quaternion rotation =
            AZ::Quaternion::CreateFromEulerAnglesDegrees(AZ::Vector3(distributions[3](gen), distributions[4](gen), distributions[5](gen)));
        transform.SetRotation(rotation);
        transform.SetUniformScale(AZStd::max(0.0f, distributions[6](gen)));
        return transform;
    }

    AZStd::unordered_map<int, AZStd::vector<TicketToSpawnPair>> PrepareTicketsToSpawn(
        AZStd::vector<GeoJSONSpawnableEntityInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations,
        AZ::u64 defaultSeed,
        SpawnCompletionCallback spawnCompletionCallback,
        const AZStd::string& physicsSceneName,
        AZ::EntityId parentId)
    {
        auto sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Unable to get physics scene interface.");
        const auto sceneHandle = sceneInterface->GetSceneHandle(physicsSceneName);

        AZStd::unordered_map<int, AZStd::vector<TicketToSpawnPair>> groupIdToTicketsMap;

        for (auto spawnableAsset : spawnableAssetConfigurations)
        {
            spawnableAsset.second.m_spawnable.QueueLoad();
        }

        for (auto& entityToSpawn : entitiesToSpawn)
        {
            if (!spawnableAssetConfigurations.contains(entityToSpawn.m_name))
            {
                AZ_Error("GeoJSONSpawnerUtils", false, "Spawnable with name [%s] not found.", entityToSpawn.m_name.c_str());
                continue;
            }

            const auto& spawnableAssetConfiguration = spawnableAssetConfigurations.at(entityToSpawn.m_name);

            for (auto& point : entityToSpawn.m_positions)
            {
                [[maybe_unused]] std::mt19937 gen = std::mt19937(defaultSeed++);

                point *= GetRandomTransform(
                    spawnableAssetConfiguration.m_positionStdDev,
                    spawnableAssetConfiguration.m_rotationStdDev,
                    spawnableAssetConfiguration.m_scaleStdDev,
                    gen);

                if (spawnableAssetConfiguration.m_placeOnTerrain)
                {
                    constexpr float maxRaycastDistance = 1000.0f;
                    const AZStd::optional<AZ::Vector3> hitPosition =
                        RaytraceTerrain(point.GetTranslation(), sceneHandle, -AZ::Vector3::CreateAxisZ(), maxRaycastDistance);

                    if (hitPosition.has_value())
                    {
                        point.SetTranslation(hitPosition.value());
                    }
                    else
                    {
                        continue;
                    }
                }

                AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
                AzFramework::EntitySpawnTicket ticket(spawnableAssetConfiguration.m_spawnable);
                optionalArgs.m_preInsertionCallback = [point]([[maybe_unused]] auto id, auto view)
                {
                    if (view.empty())
                    {
                        return;
                    }
                    AZ::Entity* root = *view.begin();
                    AZ_Assert(root, "Invalid root entity.");

                    auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
                    transformInterface->SetWorldTM(point);
                };

                optionalArgs.m_completionCallback = [parentId, spawnCompletionCallback]([[maybe_unused]] auto id, auto view)
                {
                    spawnCompletionCallback();
                    if (view.empty())
                    {
                        return;
                    }

                    const AZ::Entity* root = *view.begin();
                    AZ::TransformBus::Event(root->GetId(), &AZ::TransformBus::Events::SetParent, parentId);
                };

                optionalArgs.m_priority = AzFramework::SpawnablePriority_Lowest;

                if (!groupIdToTicketsMap.contains(entityToSpawn.m_id))
                {
                    groupIdToTicketsMap[entityToSpawn.m_id] = {};
                }
                groupIdToTicketsMap.at(entityToSpawn.m_id).emplace_back(AZStd::make_pair(AZStd::move(ticket), AZStd::move(optionalArgs)));
            }
        }

        return groupIdToTicketsMap;
    }

    AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> SpawnEntities(
        AZStd::unordered_map<int, AZStd::vector<TicketToSpawnPair>>& ticketsToSpawn)
    {
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawnable entities definition.");

        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> groupIdToTicketsMap;
        for (auto& groupIdToSpawn : ticketsToSpawn)
        {
            if (!groupIdToTicketsMap.contains(groupIdToSpawn.first))
            {
                groupIdToTicketsMap[groupIdToSpawn.first] = {};
            }
            for (auto& ticketToSpawn : groupIdToSpawn.second)
            {
                spawner->SpawnAllEntities(ticketToSpawn.first, ticketToSpawn.second);
                groupIdToTicketsMap.at(groupIdToSpawn.first).emplace_back(AZStd::move(ticketToSpawn.first));
            }
        }

        return groupIdToTicketsMap;
    }

    void DespawnEntity(AzFramework::EntitySpawnTicket& ticket, DespawnCallback callback)
    {
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawnable entities definition.");
        AzFramework::DespawnAllEntitiesOptionalArgs optionalArgs;
        optionalArgs.m_completionCallback = [callback](auto id)
        {
            callback(id);
        };
        spawner->DespawnAllEntities(ticket, optionalArgs);
    }

    AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration> GetSpawnableAssetFromVector(
        const AZStd::vector<GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations)
    {
        AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration> spawnableAssetMap;
        for (auto& spawnableAssetConfigurationElement : spawnableAssetConfigurations)
        {
            spawnableAssetMap[spawnableAssetConfigurationElement.m_name] = spawnableAssetConfigurationElement;
        }

        return spawnableAssetMap;
    }

    AZStd::vector<GeoJSONSpawnableEntityInfo> GetSpawnableEntitiesFromFeatureObjectVector(
        const AZStd::vector<FeatureObjectInfo>& featureObjects,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations)
    {
        if (!ROS2::GeoreferenceRequestsBus::HasHandlers())
        {
            AZ_Error("GeoJSONSpawnerUtils", false, "Cannot convert WGS84 coordinates - Level is not geographically positioned.");
            return {};
        }

        AZStd::vector<GeoJSONSpawnableEntityInfo> spawnableEntities;
        for (const auto& featureObjectInfo : featureObjects)
        {
            if (!spawnableAssetConfigurations.contains(featureObjectInfo.m_name))
            {
                AZ_Error("GeoJSONSpawnerUtils", false, "Spawnable with name [%s] not found.", featureObjectInfo.m_name.c_str());
                continue;
            }
            const auto& spawnableAssetConfig = spawnableAssetConfigurations.at(featureObjectInfo.m_name);
            GeoJSONSpawnableEntityInfo spawnableEntityInfo;
            spawnableEntityInfo.m_name = featureObjectInfo.m_name;
            spawnableEntityInfo.m_id = featureObjectInfo.m_id;
            for (const auto& point : featureObjectInfo.m_coordinates)
            {
                constexpr float defaultScale = 1.0f;
                const AZ::Quaternion rotation = AZ::Quaternion::CreateIdentity();
                ROS2::WGS::WGS84Coordinate coordinate;
                AZ::Vector3 coordinateInLevel = AZ::Vector3(-1);
                coordinate.m_longitude = point[0];
                coordinate.m_latitude = point[1];
                coordinate.m_altitude = spawnableAssetConfig.m_raytraceStartingHeight;

                ROS2::GeoreferenceRequestsBus::BroadcastResult(
                    coordinateInLevel, &ROS2::GeoreferenceRequestsBus::Events::ConvertFromWGS84ToLevel, coordinate);

                AZ::Transform transform{ coordinateInLevel, rotation, defaultScale };
                spawnableEntityInfo.m_positions.emplace_back(AZStd::move(transform));
            }
            spawnableEntities.push_back(spawnableEntityInfo);
        }

        return spawnableEntities;
    }

    bool ValidateGeoJSON(const rapidjson::Document& geoJsonDocument)
    {
        rapidjson::Document schemaDocument;
        if (schemaDocument.Parse(GeoJSONSchema).HasParseError())
        {
            AZ_Error("GeoJSONSpawner", false, "Unable to parse schema.");
            return false;
        }

        rapidjson::SchemaDocument schema(schemaDocument);
        rapidjson::SchemaValidator validator(schema);

        if (!geoJsonDocument.Accept(validator))
        {
            rapidjson::StringBuffer buffer;
            validator.GetInvalidSchemaPointer().StringifyUriFragment(buffer);
            AZ_Error(
                "GeoJSONSpawner", false, "Invalid code: %s. Invalid key: %s.", buffer.GetString(), validator.GetInvalidSchemaKeyword());
            return false;
        }

        return true;
    }

    Coordinates ExtractPoints(const rapidjson::Value& geometry)
    {
        Coordinates spawnableCoordinates;

        AZStd::string geometryTypeStr = geometry["type"].GetString();
        AZStd::to_lower(geometryTypeStr.begin(), geometryTypeStr.end());
        const GeometryType geometryType = GetGeometryType(geometryTypeStr);

        // GeometryCollection is handled at the beginning, as it has a slightly different structure
        // and in place of `coordinates` field it has `geometries` field, which needs to be extracted
        // to get `coordinates` field from associated objects
        if (geometryType == GeometryType::GeometryCollection)
        {
            const rapidjson::Value& geometries = geometry["geometries"];
            for (const auto& geometry : geometries.GetArray())
            {
                Coordinates geometryCoordinates = ExtractPoints(geometry);
                spawnableCoordinates.insert(spawnableCoordinates.end(), geometryCoordinates.begin(), geometryCoordinates.end());
            }

            return spawnableCoordinates;
        }

        const rapidjson::Value& coordinates = geometry["coordinates"];
        if (geometryType == GeometryType::Point)
        {
            spawnableCoordinates.push_back({ coordinates[0].GetDouble(), coordinates[1].GetDouble(), 0.0f });
        }
        else if (geometryType == GeometryType::LineString || geometryType == GeometryType::MultiPoint)
        {
            for (const auto& point : coordinates.GetArray())
            {
                spawnableCoordinates.push_back({ point[0].GetDouble(), point[1].GetDouble(), 0.0f });
            }
        }
        else if (geometryType == GeometryType::Polygon || geometryType == GeometryType::MultiLineString)
        {
            for (const auto& object : coordinates.GetArray())
            {
                for (const auto& point : object.GetArray())
                {
                    spawnableCoordinates.push_back({ point[0].GetDouble(), point[1].GetDouble(), 0.0f });
                }

                if (geometryType == GeometryType::Polygon)
                {
                    spawnableCoordinates.pop_back();
                }
            }
        }
        else if (geometryType == GeometryType::MultiPolygon)
        {
            for (const auto& multiPolygon : coordinates.GetArray())
            {
                for (const auto& object : multiPolygon.GetArray())
                {
                    for (const auto& point : object.GetArray())
                    {
                        spawnableCoordinates.push_back({ point[0].GetDouble(), point[1].GetDouble(), 0.0f });
                    }

                    spawnableCoordinates.pop_back();
                }
            }
        }
        else if (geometryType == GeometryType::Unknown)
        {
            AZ_Error("GeoJSONSpawner", false, "Unknown geometry type.");
        }

        return spawnableCoordinates;
    }

    Ids ExtractIds(const rapidjson::Document& geoJsonDocument)
    {
        AZStd::unordered_set<int> ids;
        if (!ValidateGeoJSON(geoJsonDocument))
        {
            AZ_Error("GeoJSONSpawner", false, "Failed to validate JSON string.");
            return ids;
        }

        const rapidjson::Value& featureCollection = geoJsonDocument["features"];
        for (const auto& feature : featureCollection.GetArray())
        {
            const rapidjson::Value& properties = feature["properties"];
            ids.insert(properties["id"].GetInt());
        }

        return ids;
    }

    Ids ExtractIdsFromRawString(const AZStd::string& rawGeoJson)
    {
        auto loadResult = AZ::JsonSerializationUtils::ReadJsonString(rawGeoJson);

        if (!loadResult.IsSuccess())
        {
            AZ_Error("GeoJSONSpawnerUtils", false, "%s", loadResult.GetError().c_str());
            return {};
        }

        return ExtractIds(loadResult.GetValue<rapidjson::Document>());
    }

    AZStd::vector<FeatureObjectInfo> ParseGeoJSON(const rapidjson::Document& geoJsonDocument)
    {
        AZStd::vector<FeatureObjectInfo> spawnableInfoContainer;

        if (!ValidateGeoJSON(geoJsonDocument))
        {
            AZ_Error("GeoJSONSpawner", false, "Failed to validate JSON string.");
            return spawnableInfoContainer;
        }

        const rapidjson::Value& featureCollection = geoJsonDocument["features"];
        for (const auto& feature : featureCollection.GetArray())
        {
            const rapidjson::Value& properties = feature["properties"];
            const rapidjson::Value& geometry = feature["geometry"];
            FeatureObjectInfo featureObjectInfo;
            featureObjectInfo.m_name = properties["spawnable_name"].GetString();
            featureObjectInfo.m_id = properties["id"].GetInt();
            featureObjectInfo.m_coordinates = ExtractPoints(geometry);

            spawnableInfoContainer.push_back(featureObjectInfo);
        }

        return spawnableInfoContainer;
    }

    AZStd::vector<FeatureObjectInfo> ParseJSONFromFile(const AZStd::string& filePath)
    {
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance();
        if (!fileIO || !fileIO->Exists(filePath.c_str()))
        {
            AZ_Error("GeoJSONSpawnerUtils", false, "Cannot find file %s", filePath.c_str());
            return {};
        }

        auto loadResult = AZ::JsonSerializationUtils::ReadJsonFile(filePath);

        if (!loadResult.IsSuccess())
        {
            AZ_Error("GeoJSONSpawnerUtils", false, "%s", loadResult.GetError().c_str());
            return {};
        }

        return ParseGeoJSON(loadResult.GetValue<rapidjson::Document>());
    }

    AZStd::vector<FeatureObjectInfo> ParseJSONFromRawString(const AZStd::string& rawGeoJson)
    {
        auto loadResult = AZ::JsonSerializationUtils::ReadJsonString(rawGeoJson);

        if (!loadResult.IsSuccess())
        {
            AZ_Error("GeoJSONSpawnerUtils", false, "%s", loadResult.GetError().c_str());
            return {};
        }

        return ParseGeoJSON(loadResult.GetValue<rapidjson::Document>());
    }

    GeometryType GetGeometryType(const AZStd::string& geometryType)
    {
        if (geometryType == "point")
        {
            return GeometryType::Point;
        }
        if (geometryType == "multipoint")
        {
            return GeometryType::MultiPoint;
        }
        if (geometryType == "linestring")
        {
            return GeometryType::LineString;
        }
        if (geometryType == "multilinestring")
        {
            return GeometryType::MultiLineString;
        }
        if (geometryType == "polygon")
        {
            return GeometryType::Polygon;
        }
        if (geometryType == "multipolygon")
        {
            return GeometryType::MultiPolygon;
        }
        if (geometryType == "geometrycollection")
        {
            return GeometryType::GeometryCollection;
        }

        return GeometryType::Unknown;
    }

    bool IsTerrainAvailable()
    {
        return AzFramework::Terrain::TerrainDataRequestBus::HasHandlers();
    }
} // namespace GeoJSONSpawner::GeoJSONUtils
