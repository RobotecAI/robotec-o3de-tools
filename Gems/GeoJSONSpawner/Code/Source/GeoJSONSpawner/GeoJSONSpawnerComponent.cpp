
#include "GeoJSONSpawnerComponent.h"

#include "Schemas/GeoJSONSchema.h"
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <ROS2/Georeference/GeoreferenceBus.h>
#include <ROS2/Georeference/GeoreferenceStructures.h>

#include <AzCore/Serialization/EditContext.h>
#include <fstream>
#include <iostream>

#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/JSON/document.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <bits/fs_fwd.h>
#include <rapidjson/schema.h>

namespace GeoJSONSpawner
{
    void GeoJSONSpawnerComponent::Activate()
    {
        GeoJSONSpawnerRequestBus::Handler::BusConnect(GetEntityId());
    }

    void GeoJSONSpawnerComponent::Deactivate()
    {
        GeoJSONSpawnerRequestBus::Handler::BusDisconnect();
    }

    void GeoJSONSpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerComponent, AZ::Component>()
                ->Version(0)
                ->Field("SpawnableAssets", &GeoJSONSpawnerComponent::m_spawnableAssets)
                ->Field("Altitude", &GeoJSONSpawnerComponent::m_altitude);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GeoJSONSpawnerComponent>("GeoJSONSpawnerComponent", "GeoJSON Spawner component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &GeoJSONSpawnerComponent::m_spawnableAssets, "SpawnableAssets", "Spawnable assets")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GeoJSONSpawnerComponent::m_altitude, "Altitude", "Altitude");
            }
        }
    }

    void GeoJSONSpawnerComponent::Spawn(const AZStd::string& rawJsonString)
    {
        const auto parentId = GetEntityId();
        auto result = ParseGeoJSON(rawJsonString);

        m_spawnableTickets.clear();

        for (auto& nameSpawnablePair : m_spawnableAssets)
        {
            nameSpawnablePair.second.QueueLoad();
        }

        auto sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Unable to get physics scene interface.");

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawner.");

        for (const auto& element : result)
        {
            if (!m_spawnableAssets.contains(element.first))
            {
                AZ_Error("GeoJSONSpawner", false, "Spawnable with name [%s] not found.", element.first.c_str());
                continue;
            }

            const auto& spawnable = m_spawnableAssets.at(element.first);

            for (const auto& point : element.second)
            {
                AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
                AzFramework::EntitySpawnTicket ticket(spawnable);

                AZ::Transform transform;
                ROS2::WGS::WGS84Coordinate coordinate;
                AZ::Vector3 coordinateInLevel = AZ::Vector3(-1);
                AZ::Quaternion rotation = AZ::Quaternion::CreateIdentity();
                coordinate.m_longitude = point[0];
                coordinate.m_latitude = point[1];
                coordinate.m_altitude = m_altitude;

                ROS2::GeoreferenceRequestsBus::BroadcastResult(
                    coordinateInLevel, &ROS2::GeoreferenceRequests::ConvertFromWGS84ToLevel, coordinate);

                transform = { coordinateInLevel, rotation, 1.0f };

                optionalArgs.m_preInsertionCallback = [transform]([[maybe_unused]] auto id, auto view)
                {
                    if (view.empty())
                    {
                        return;
                    }
                    AZ::Entity* root = *view.begin();

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

                spawner->SpawnAllEntities(ticket, optionalArgs);
                m_spawnableTickets[ticket.GetId()] = AZStd::move(ticket);
            }
        }
    }

    bool GeoJSONSpawnerComponent::ValidateGeoJSON(const rapidjson::Document& geoJsonDocument)
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

    GeoJSONSpawnerComponent::Coordinates GeoJSONSpawnerComponent::ExtractPoints(const rapidjson::Value& geometry)
    {
        Coordinates spawnableCoordinates;

        AZStd::string geometryTypeStr = geometry["type"].GetString();
        AZStd::to_lower(geometryTypeStr.begin(), geometryTypeStr.end());
        const GeometryType geometryType = GetGeometryType(geometryTypeStr);
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

    GeoJSONSpawnerComponent::SpawnableCoordinatesMap GeoJSONSpawnerComponent::ParseGeoJSON(const AZStd::string& rawJsonString)
    {
        SpawnableCoordinatesMap spawnableCoordinatesMap;
        rapidjson::Document document;

        if (document.Parse(rawJsonString.c_str()).HasParseError())
        {
            AZ_Error("GeoJSONSpawner", false, "Failed to parse JSON string.");
            return spawnableCoordinatesMap;
        }

        if (!ValidateGeoJSON(document))
        {
            AZ_Error("GeoJSONSpawner", false, "Failed to validate JSON string.");
            return spawnableCoordinatesMap;
        }

        const rapidjson::Value& featureCollection = document["features"];
        for (const auto& feature : featureCollection.GetArray())
        {
            const rapidjson::Value& properties = feature["properties"];
            AZStd::string spawnableName;
            spawnableName = properties["spawnable_name"].GetString();
            const rapidjson::Value& geometry = feature["geometry"];
            const Coordinates extractedPoints = ExtractPoints(geometry);
            if (auto it = spawnableCoordinatesMap.find(spawnableName); it != spawnableCoordinatesMap.end())
            {
                auto& spawnableCoordinates = it->second;
                spawnableCoordinates.insert(spawnableCoordinates.end(), extractedPoints.begin(), extractedPoints.end());
            }
            else
            {
                spawnableCoordinatesMap[spawnableName] = extractedPoints;
            }
        }

        return spawnableCoordinatesMap;
    }

    GeometryType GeoJSONSpawnerComponent::GetGeometryType(const AZStd::string& geometryType)
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
} // namespace GeoJSONSpawner
