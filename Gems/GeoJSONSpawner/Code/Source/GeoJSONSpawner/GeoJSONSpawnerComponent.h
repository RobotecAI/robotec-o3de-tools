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
#include "GeoJSONSpawnerUtils.h"
#include "GeoJSONSpawner/GeoJSONSpawnerBus.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzFramework/Terrain/TerrainDataRequestBus.h>

namespace GeoJSONSpawner
{
    enum class SpawnerState
    {
        Idle = 0,
        Spawning,
        Despawning
    };
    //! Game component that spawns entities from a GeoJSON.
    //! This component is used to spawn various spawnables, on the first tick.
    class GeoJSONSpawnerComponent
        : public AZ::Component
        , public GeoJSONSpawnerRequestBus::Handler
        , public AZ::TickBus::Handler
        , protected AzFramework::Terrain::TerrainDataNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(GeoJSONSpawnerComponent, GeoJSONSpawnerComponentTypeId);

        GeoJSONSpawnerComponent() = default;
        explicit GeoJSONSpawnerComponent(
            const AZStd::unordered_map<AZStd::string, GeoJSONUtils::GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations,
            const AZ::IO::Path& geoJsonFilePath,
            AZ::u64 defaultSeed);
        ~GeoJSONSpawnerComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

        // GeoJSONSpawnerRequestBus::Handler overrides...
        Result SpawnWithRawString(const AZStd::string& rawJsonString) override;
        Result SpawnWithAssetPath(const AZ::IO::Path& assetPath) override;
        Result Modify(const AZStd::string& rawJsonString) override;
        Result DeleteAll() override;
        Result DeleteById(const AZStd::unordered_set<int>& idsToDelete) override;
        GetIdsResult GetIds() const override;

        // AzFramework::Terrain::TerrainDataNotificationBus overrides
        void OnTerrainDataCreateEnd() override;
        void OnTerrainDataDestroyBegin() override;

    private:
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void SpawnEntities();
        void SpawnEntities(const AZStd::vector<GeoJSONUtils::FeatureObjectInfo>& featureObjectsToSpawn);
        void SpawnCachedEntities(const AZStd::vector<GeoJSONUtils::FeatureObjectInfo>& cachedObjectsToSpawn);

        void FillGroupIdToTicketIdMap();
        void FillGroupIdToTicketIdMap(const AZStd::unordered_set<int>& groupIds);

        unsigned int CountTicketsToSpawn(
            const AZStd::unordered_map<int, AZStd::vector<GeoJSONUtils::TicketToSpawnPair>>& ticketsToSpawn) const;

        void DespawnAllEntities();
        void DespawnEntitiesById(const GeoJSONUtils::Ids& ids);
        void Despawn(AzFramework::EntitySpawnTicket& ticketToDespawn);

        AZStd::unordered_map<AZStd::string, GeoJSONUtils::GeoJSONSpawnableAssetConfiguration> m_spawnableAssetConfigurations;
        AZ::u64 m_defaultSeed;
        AZ::IO::Path m_geoJsonFilePath;
        AZStd::vector<GeoJSONUtils::FeatureObjectInfo> m_cachedObjectsToSpawn;

        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> m_spawnableTickets;
        AZStd::unordered_map<int, AZStd::unordered_set<AzFramework::EntitySpawnTicket::Id>> m_spawnableTicketsIds;
        AZStd::atomic<unsigned int> m_ticketsToDespawn{ 0 };
        AZStd::atomic<unsigned int> m_ticketsToSpawn{ 0 };
        AZStd::vector<GeoJSONUtils::GeoJSONSpawnableEntityInfo> m_spawnableEntityInfo;
        GeoJSONUtils::SpawnStatus m_despawnStatus = GeoJSONUtils::SpawnStatus::Success;

        SpawnerState m_spawnerState{ SpawnerState::Idle };
        AZStd::queue<SpawnerState> m_spawnerStateQueue;

        // Terrain notify
        bool m_terrainCreatedOnlyOnce{ false }; //!< Is terrain fully generated once
    };
} // namespace GeoJSONSpawner
