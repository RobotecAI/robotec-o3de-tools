/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerComponent.h"

#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsScene.h>

namespace GeoJSONSpawner
{
    GeoJSONSpawnerComponent::GeoJSONSpawnerComponent(
        const AZStd::unordered_map<AZStd::string, GeoJSONUtils::GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations,
        const AZStd::string& geoJsonFilePath,
        AZ::u64 defaultSeed)
        : m_spawnableAssetConfigurations(spawnableAssetConfigurations)
        , m_geoJsonFilePath(geoJsonFilePath)
        , m_defaultSeed(defaultSeed)
    {
    }

    void GeoJSONSpawnerComponent::Activate()
    {
        AZ::TickBus::QueueFunction(
            [this]()
            {
                SpawnEntities();
            });

        GeoJSONSpawnerRequestBus::Handler::BusConnect(GetEntityId());
    }

    void GeoJSONSpawnerComponent::Deactivate()
    {
        GeoJSONSpawnerRequestBus::Handler::BusDisconnect();
    }

    void GeoJSONSpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        GeoJSONUtils::GeoJSONSpawnableAssetConfiguration::Reflect(context);
        GeoJSONUtils::GeoJSONSpawnableEntityInfo::Reflect(context);
        GeoJSONUtils::GeometryObjectInfo::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerComponent, AZ::Component>()
                ->Version(0)
                ->Field("SpawnableAssetConfigurations", &GeoJSONSpawnerComponent::m_spawnableAssetConfigurations)
                ->Field("GeoJsonFilePath", &GeoJSONSpawnerComponent::m_geoJsonFilePath)
                ->Field("DefaultSeed", &GeoJSONSpawnerComponent::m_defaultSeed);
        }
    }

    void GeoJSONSpawnerComponent::SpawnEntities()
    {
        m_spawnableTickets.clear();
        const auto geometryObjectInfo = GeoJSONUtils::ParseJSONFromFile(m_geoJsonFilePath.c_str());
        m_spawnableEntityInfo =
            GeoJSONUtils::GetSpawnableEntitiesFromGeometryObjectVector(geometryObjectInfo, m_spawnableAssetConfigurations);
        m_spawnableTickets = GeoJSONUtils::SpawnEntities(
            m_spawnableEntityInfo, m_spawnableAssetConfigurations, m_defaultSeed, AzPhysics::DefaultPhysicsSceneName, GetEntityId());
    }

    void GeoJSONSpawnerComponent::Spawn(const AZStd::string& rawJsonString)
    {
        const auto result = GeoJSONUtils::ParseJSONFromRawString(rawJsonString);
        m_spawnableEntityInfo = GeoJSONUtils::GetSpawnableEntitiesFromGeometryObjectVector(result, m_spawnableAssetConfigurations);
        m_spawnableTickets.clear();
        m_spawnableTickets = GeoJSONUtils::SpawnEntities(
            m_spawnableEntityInfo, m_spawnableAssetConfigurations, m_defaultSeed, AzPhysics::EditorPhysicsSceneName, GetEntityId());
    }
} // namespace GeoJSONSpawner
