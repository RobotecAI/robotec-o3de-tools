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
#include <rapidjson/schema.h>

namespace GeoJSONSpawner::GeoJSONUtils
{
    void GeoJSONSpawnerConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerConfiguration>()
                ->Version(0)
                ->Field("GeoJSONAssetId", &GeoJSONSpawnerConfiguration::m_geoJsonAssetId)
                ->Field("SpawnableAssets", &GeoJSONSpawnerConfiguration::m_spawnableAssets)
                ->Field("Altitude", &GeoJSONSpawnerConfiguration::m_altitude);
            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GeoJSONSpawnerConfiguration>("GeoJSONSpawnerConfiguration", "GeoJSON Spawner Configuration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &GeoJSONSpawnerConfiguration::m_geoJsonAssetId, "GeoJSON asset", "GeoJSON asset")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerConfiguration::m_spawnableAssets,
                        "SpawnableAssets",
                        "Spawnable Assets")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GeoJSONSpawnerConfiguration::m_altitude, "Altitude", "Altitude");
            }
        }
    }

    AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> SpawnEntities(
        const AZStd::vector<GeometryObjectInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>>& spawnableAssetConfiguration,
        AZ::EntityId parentId)
    {
        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> groupIdToTicketsMap;
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawnable entities definition.");

        for (auto spawnablePair : spawnableAssetConfiguration)
        {
            spawnablePair.second.QueueLoad();
        }

        for (const auto& entityToSpawn : entitiesToSpawn)
        {
            if (!spawnableAssetConfiguration.contains(entityToSpawn.m_name))
            {
                AZ_Error("GeoJSONSpawnerUtils", false, "Spawnable with name [%s] not found.", entityToSpawn.m_name.c_str());
                continue;
            }

            const auto& spawnable = spawnableAssetConfiguration.at(entityToSpawn.m_name);
            if (!groupIdToTicketsMap.contains(entityToSpawn.m_id))
            {
                groupIdToTicketsMap[entityToSpawn.m_id] = {};
            }

            for (const auto& point : entityToSpawn.m_coordinates)
            {
                AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
                AzFramework::EntitySpawnTicket ticket(spawnable);

                AZ::Transform transform = AZ::Transform::CreateIdentity();
                ROS2::WGS::WGS84Coordinate coordinate;
                AZ::Vector3 coordinateInLevel = AZ::Vector3(-1);
                AZ::Quaternion rotation = AZ::Quaternion::CreateIdentity();
                coordinate.m_longitude = point[0];
                coordinate.m_latitude = point[1];
                coordinate.m_altitude = 145.0;

                ROS2::GeoreferenceRequestsBus::BroadcastResult(
                    coordinateInLevel, &ROS2::GeoreferenceRequestsBus::Events::ConvertFromWGS84ToLevel, coordinate);

                transform = { coordinateInLevel, rotation, 1.0f };

                optionalArgs.m_preInsertionCallback = [transform]([[maybe_unused]] auto id, auto view)
                {
                    if (view.empty())
                    {
                        return;
                    }
                    AZ::Entity* root = *view.begin();
                    AZ_Assert(root, "Invalid root entity.");

                    auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
                    transformInterface->SetWorldTM(transform);
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
            spawnableCoordinates.push_back({ coordinates[0].GetDouble(), coordinates[1].GetDouble() });
        }
        else if (geometryType == GeometryType::LineString || geometryType == GeometryType::MultiPoint)
        {
            for (const auto& point : coordinates.GetArray())
            {
                spawnableCoordinates.push_back({ point[0].GetDouble(), point[1].GetDouble() });
            }
        }
        else if (geometryType == GeometryType::Polygon || geometryType == GeometryType::MultiLineString)
        {
            for (const auto& object : coordinates.GetArray())
            {
                for (const auto& point : object.GetArray())
                {
                    spawnableCoordinates.push_back({ point[0].GetDouble(), point[1].GetDouble() });
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
                        spawnableCoordinates.push_back({ point[0].GetDouble(), point[1].GetDouble() });
                    }

                    spawnableCoordinates.pop_back();
                }
            }
        }
        else if (geometryType == GeometryType::GeometryCollection)
        {
            const rapidjson::Value& geometries = geometry["geometries"];
            for (const auto& geometry : geometries.GetArray())
            {
                Coordinates geometryCoordinates = ExtractPoints(geometry);
                spawnableCoordinates.insert(spawnableCoordinates.end(), geometryCoordinates.begin(), geometryCoordinates.end());
            }
        }
        else if (geometryType == GeometryType::Unknown)
        {
            AZ_Error("GeoJSONSpawner", false, "Unknown geometry type.");
        }

        return spawnableCoordinates;
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