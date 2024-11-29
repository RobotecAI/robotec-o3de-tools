/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerUtils.h"
#include "Schemas/GeoJSONSchema.h"

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
    void GeometryObjectInfo::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeometryObjectInfo>()
                ->Version(0)
                ->Field("Id", &GeometryObjectInfo::m_id)
                ->Field("Name", &GeometryObjectInfo::m_name)
                ->Field("Coordinates", &GeometryObjectInfo::m_coordinates);
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
                ->Field("RaycastStartingHeight", &GeoJSONSpawnableAssetConfiguration::m_raycastStartingHeight);

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
                        "Position std dev",
                        "Position std dev")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_rotationStdDev,
                        "Rotation std dev",
                        "Rotation std dev")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &GeoJSONSpawnableAssetConfiguration::m_scaleStdDev, "Scale std dev", "Scale std dev")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_placeOnTerrain,
                        "Place on terrain",
                        "Place on terrain")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnableAssetConfiguration::m_raycastStartingHeight,
                        "Raycast starting height",
                        "Raycast starting height");
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
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(physicsSystem, "Unable to get physics system interface");
        AZ_Assert(sceneInterface, "Unable to get physics scene interface");

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

    AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> SpawnEntities(
        AZStd::vector<GeoJSONSpawnableEntityInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations,
        AZ::u64 defaultSeed,
        const AZStd::string& physicsSceneName,
        AZ::EntityId parentId)
    {
        auto sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Unable to get physics scene interface");
        const auto sceneHandle = sceneInterface->GetSceneHandle(physicsSceneName);

        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> groupIdToTicketsMap;
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawnable entities definition.");

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
            if (!groupIdToTicketsMap.contains(entityToSpawn.m_id))
            {
                groupIdToTicketsMap[entityToSpawn.m_id] = {};
            }

            for (auto& point : entityToSpawn.m_positions)
            {
                AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
                AzFramework::EntitySpawnTicket ticket(spawnableAssetConfiguration.m_spawnable);

                [[maybe_unused]] std::mt19937 gen = std::mt19937(defaultSeed++);

                AZ::Quaternion rotation = AZ::Quaternion::CreateIdentity();
                AZ::Transform pointTransform{ point, rotation, 1.0f };
                pointTransform *= GetRandomTransform(
                    spawnableAssetConfiguration.m_positionStdDev,
                    spawnableAssetConfiguration.m_rotationStdDev,
                    spawnableAssetConfiguration.m_scaleStdDev,
                    gen);

                if (spawnableAssetConfiguration.m_placeOnTerrain)
                {
                    const AZStd::optional<AZ::Vector3> hitPosition =
                        RaytraceTerrain(pointTransform.GetTranslation(), sceneHandle, -AZ::Vector3::CreateAxisZ(), 1000.0f);

                    if (hitPosition.has_value())
                    {
                        pointTransform.SetTranslation(hitPosition.value());
                    }
                    else
                    {
                        continue;
                    }
                }
                point = pointTransform.GetTranslation(); // Update point coords to use it to show labels

                optionalArgs.m_preInsertionCallback = [pointTransform]([[maybe_unused]] auto id, auto view)
                {
                    if (view.empty())
                    {
                        return;
                    }
                    AZ::Entity* root = *view.begin();
                    AZ_Assert(root, "Invalid root entity.");

                    auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
                    transformInterface->SetWorldTM(pointTransform);
                };

                optionalArgs.m_completionCallback = [parentId]([[maybe_unused]] auto id, auto view)
                {
                    if (view.empty())
                    {
                        return;
                    }

                    const AZ::Entity* root = *view.begin();
                    AZ::TransformBus::Event(root->GetId(), &AZ::TransformBus::Events::SetParent, parentId);
                };

                optionalArgs.m_priority = AzFramework::SpawnablePriority_Lowest;
                spawner->SpawnAllEntities(ticket, optionalArgs);
                groupIdToTicketsMap.at(entityToSpawn.m_id).push_back(AZStd::move(ticket));
            }
        }

        return groupIdToTicketsMap;
    }

    void DespawnEntity(AzFramework::EntitySpawnTicket& ticket, DespawnCallback callback)
    {
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
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

    AZStd::vector<GeoJSONSpawnableEntityInfo> GetSpawnableEntitiesFromGeometryObjectVector(
        const AZStd::vector<GeometryObjectInfo>& geometryObjects,
        const AZStd::unordered_map<AZStd::string, GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations)
    {
        AZStd::vector<GeoJSONSpawnableEntityInfo> spawnableEntities;
        for (const auto& geometryObjectInfo : geometryObjects)
        {
            if (!spawnableAssetConfigurations.contains(geometryObjectInfo.m_name))
            {
                AZ_Error("GeoJSONSpawnerUtils", false, "Spawnable with name [%s] not found.", geometryObjectInfo.m_name.c_str());
                continue;
            }
            const auto& spawnableAssetConfig = spawnableAssetConfigurations.at(geometryObjectInfo.m_name);
            GeoJSONSpawnableEntityInfo spawnableEntityInfo;
            spawnableEntityInfo.m_name = geometryObjectInfo.m_name;
            spawnableEntityInfo.m_id = geometryObjectInfo.m_id;
            for (const auto& point : geometryObjectInfo.m_coordinates)
            {
                ROS2::WGS::WGS84Coordinate coordinate;
                AZ::Vector3 coordinateInLevel = AZ::Vector3(-1);
                coordinate.m_longitude = point[0];
                coordinate.m_latitude = point[1];
                coordinate.m_altitude = spawnableAssetConfig.m_raycastStartingHeight;

                ROS2::GeoreferenceRequestsBus::BroadcastResult(
                    coordinateInLevel, &ROS2::GeoreferenceRequestsBus::Events::ConvertFromWGS84ToLevel, coordinate);

                spawnableEntityInfo.m_positions.emplace_back(AZStd::move(coordinateInLevel));
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
            AZ_Error("GeoJSONSpawner", false, "Unable to parse schema");
            return false;
        }

        rapidjson::SchemaDocument schema(schemaDocument);
        rapidjson::SchemaValidator validator(schema);

        if (!geoJsonDocument.Accept(validator))
        {
            rapidjson::StringBuffer buffer;
            validator.GetInvalidSchemaPointer().StringifyUriFragment(buffer);
            AZ_Error("GeoJSONSpawner", false, "Invalid code: %s. Invalid key: %s", buffer.GetString(), validator.GetInvalidSchemaKeyword());
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

    AZStd::vector<GeometryObjectInfo> ParseGeoJSON(const rapidjson::Document& geoJsonDocument)
    {
        AZStd::vector<GeometryObjectInfo> spawnableInfoContainer;

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
            GeometryObjectInfo geometryObjectInfo;
            geometryObjectInfo.m_name = properties["spawnable_name"].GetString();
            ;
            geometryObjectInfo.m_id = properties["id"].GetInt();
            geometryObjectInfo.m_coordinates = ExtractPoints(geometry);

            spawnableInfoContainer.push_back(geometryObjectInfo);
        }

        return spawnableInfoContainer;
    }

    AZStd::vector<GeometryObjectInfo> ParseJSONFromFile(const AZStd::string& filePath)
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

    AZStd::vector<GeometryObjectInfo> ParseJSONFromRawString(const AZStd::string& rawGeoJson)
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

} // namespace GeoJSONSpawner::GeoJSONUtils
