/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
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
        const AZ::IO::Path& geoJsonFilePath,
        AZ::u64 defaultSeed)
        : m_spawnableAssetConfigurations(spawnableAssetConfigurations)
        , m_geoJsonFilePath(geoJsonFilePath)
        , m_defaultSeed(defaultSeed)
    {
    }

    void GeoJSONSpawnerComponent::Activate()
    {
        if (GeoJSONUtils::IsTerrainAvailable())
        {
            AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusConnect();
        }
        else
        {
            OnTerrainDataCreateEnd();
        }

        GeoJSONSpawnerRequestBus::Handler::BusConnect(GetEntityId());
    }

    void GeoJSONSpawnerComponent::Deactivate()
    {
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }

        OnTerrainDataDestroyBegin();
        GeoJSONSpawnerRequestBus::Handler::BusDisconnect();
    }

    void GeoJSONSpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        GeoJSONUtils::GeoJSONSpawnableAssetConfiguration::Reflect(context);
        GeoJSONUtils::GeoJSONSpawnableEntityInfo::Reflect(context);
        GeoJSONUtils::FeatureObjectInfo::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Enum<GeoJSONUtils::SpawnDespawnStatus>()
                ->Version(0)
                ->Value("Success", GeoJSONUtils::SpawnDespawnStatus::Success)
                ->Value("Fail", GeoJSONUtils::SpawnDespawnStatus::Fail)
                ->Value("Stopped", GeoJSONUtils::SpawnDespawnStatus::Stopped)
                ->Value("Warning", GeoJSONUtils::SpawnDespawnStatus::Warning)
                ->Value("Invalid", GeoJSONUtils::SpawnDespawnStatus::Invalid);

            serializeContext->Class<GeoJSONSpawnerComponent, AZ::Component>()
                ->Version(0)
                ->Field("SpawnableAssetConfigurations", &GeoJSONSpawnerComponent::m_spawnableAssetConfigurations)
                ->Field("GeoJsonFilePath", &GeoJSONSpawnerComponent::m_geoJsonFilePath)
                ->Field("DefaultSeed", &GeoJSONSpawnerComponent::m_defaultSeed);
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EnumProperty<static_cast<int>(GeoJSONUtils::SpawnDespawnStatus::Success)>("SpawnStatus_Success");
            behaviorContext->EnumProperty<static_cast<int>(GeoJSONUtils::SpawnDespawnStatus::Fail)>("SpawnStatus_Fail");
            behaviorContext->EnumProperty<static_cast<int>(GeoJSONUtils::SpawnDespawnStatus::Stopped)>("SpawnStatus_Stopped");
            behaviorContext->EnumProperty<static_cast<int>(GeoJSONUtils::SpawnDespawnStatus::Warning)>("SpawnStatus_Warning");
            behaviorContext->EnumProperty<static_cast<int>(GeoJSONUtils::SpawnDespawnStatus::Invalid)>("SpawnStatus_Invalid");

            behaviorContext->EBus<GeoJSONSpawner::GeoJSONSpawnerNotificationBus>("GeoJSONSpawnerNotificationBus")
                ->Handler<GeoJSONSpawner::GeoJSONSpawnerNotificationBusHandler>();
        }
    }

    void GeoJSONSpawnerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_spawnerState == SpawnerState::Idle && m_spawnerStateQueue.empty())
        {
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }

        if (m_spawnerState == SpawnerState::Despawning)
        {
            if (m_ticketsToDespawn == 0)
            {
                m_spawnerState = SpawnerState::Idle;
            }
        }

        if (m_spawnerState == SpawnerState::Spawning)
        {
            if (!m_cachedObjectsToSpawn.empty())
            {
                SpawnCachedEntities(m_cachedObjectsToSpawn);
                m_cachedObjectsToSpawn.clear();
            }
            if (m_ticketsToSpawn == 0)
            {
                m_spawnerState = SpawnerState::Idle;
            }
        }

        if (m_spawnerState == SpawnerState::Idle && !m_spawnerStateQueue.empty())
        {
            m_spawnerState = m_spawnerStateQueue.front();
            m_spawnerStateQueue.pop();
        }
    }

    void GeoJSONSpawnerComponent::SpawnEntities()
    {
        m_spawnableTickets.clear();
        const auto featureObjectInfo = GeoJSONUtils::ParseJSONFromFile(m_geoJsonFilePath.c_str());
        SpawnEntities(featureObjectInfo);
    }

    void GeoJSONSpawnerComponent::SpawnEntities(const AZStd::vector<GeoJSONUtils::FeatureObjectInfo>& featureObjectsToSpawn)
    {
        m_spawnableEntityInfo =
            GeoJSONUtils::GetSpawnableEntitiesFromFeatureObjectVector(featureObjectsToSpawn, m_spawnableAssetConfigurations);
        auto preparedTickets = GeoJSONUtils::PrepareTicketsToSpawn(
            m_spawnableEntityInfo,
            m_spawnableAssetConfigurations,
            m_defaultSeed,
            [this]()
            {
                --m_ticketsToSpawn;
            },
            AzPhysics::DefaultPhysicsSceneName,
            GetEntityId());

        m_ticketsToSpawn = CountTicketsToSpawn(preparedTickets);

        m_spawnableTickets = GeoJSONUtils::SpawnEntities(preparedTickets);
    }

    void GeoJSONSpawnerComponent::SpawnCachedEntities(const AZStd::vector<GeoJSONUtils::FeatureObjectInfo>& cachedObjectsToSpawn)
    {
        auto entityInfos = GeoJSONUtils::GetSpawnableEntitiesFromFeatureObjectVector(cachedObjectsToSpawn, m_spawnableAssetConfigurations);

        auto preparedTickets = GeoJSONUtils::PrepareTicketsToSpawn(
            entityInfos,
            m_spawnableAssetConfigurations,
            m_defaultSeed,
            [this]()
            {
                --m_ticketsToSpawn;
            },
            AzPhysics::DefaultPhysicsSceneName,
            GetEntityId());

        m_ticketsToSpawn = CountTicketsToSpawn(preparedTickets);

        const auto spawnableTickets = GeoJSONUtils::SpawnEntities(preparedTickets);

        for (const auto& pair : spawnableTickets)
        {
            m_spawnableTickets[pair.first] = AZStd::move(pair.second);
        }
    }

    Result GeoJSONSpawnerComponent::SpawnWithRawString(const AZStd::string& rawJsonString)
    {
        if (m_spawnerState != SpawnerState::Idle || !m_spawnerStateQueue.empty())
        {
            return AZ::Failure(AZStd::string("Spawner is handling previous request. Action aborted."));
        }

        if (!m_spawnableTickets.empty())
        {
            m_spawnerStateQueue.push(SpawnerState::Despawning);
            m_spawnerStateQueue.push(SpawnerState::Spawning);
            m_cachedObjectsToSpawn = GeoJSONUtils::ParseJSONFromRawString(rawJsonString);
            DespawnAllEntities();
            AZ::TickBus::Handler::BusConnect();
        }
        else
        {
            const auto result = GeoJSONUtils::ParseJSONFromRawString(rawJsonString);
            m_spawnerStateQueue.push(SpawnerState::Spawning);
            SpawnEntities(result);
            AZ::TickBus::Handler::BusConnect();
        }

        return AZ::Success();
    }

    Result GeoJSONSpawnerComponent::SpawnWithAssetPath(const AZ::IO::Path& assetPath)
    {
        if (m_spawnerState != SpawnerState::Idle || !m_spawnerStateQueue.empty())
        {
            return AZ::Failure(AZStd::string("Spawner is handling previous request. Action aborted."));
        }

        if (!m_spawnableTickets.empty())
        {
            m_spawnerStateQueue.push(SpawnerState::Despawning);
            m_spawnerStateQueue.push(SpawnerState::Spawning);
            m_cachedObjectsToSpawn = GeoJSONUtils::ParseJSONFromFile(assetPath.c_str());
            DespawnAllEntities();
            AZ::TickBus::Handler::BusConnect();
        }
        else
        {
            const auto result = GeoJSONUtils::ParseJSONFromFile(assetPath.c_str());
            m_spawnerStateQueue.push(SpawnerState::Spawning);
            SpawnEntities(result);
            AZ::TickBus::Handler::BusConnect();
        }

        return AZ::Success();
    }

    Result GeoJSONSpawnerComponent::Modify(const AZStd::string& rawJsonString)
    {
        if (m_spawnerState != SpawnerState::Idle || !m_spawnerStateQueue.empty())
        {
            return AZ::Failure(AZStd::string("Spawner is handling previous request. Action aborted."));
        }

        const auto groupIdsToModify = GeoJSONUtils::ExtractIdsFromRawString(rawJsonString);

        for (const auto& groupId : groupIdsToModify)
        {
            if (m_spawnableTickets.find(groupId) == m_spawnableTickets.end())
            {
                return AZ::Failure(
                    AZStd::string::format("Cannot modify entities with ID: %d. Such group does not exist. Action aborted.", groupId));
            }
        }

        m_spawnerStateQueue.push(SpawnerState::Despawning);
        m_spawnerStateQueue.push(SpawnerState::Spawning);
        m_cachedObjectsToSpawn = GeoJSONUtils::ParseJSONFromRawString(rawJsonString);
        DespawnEntitiesById(groupIdsToModify);
        AZ::TickBus::Handler::BusConnect();

        return AZ::Success();
    }

    Result GeoJSONSpawnerComponent::DeleteById(const AZStd::unordered_set<int>& idsToDelete)
    {
        if (m_spawnerState != SpawnerState::Idle || !m_spawnerStateQueue.empty())
        {
            return AZ::Failure(AZStd::string("Spawner is handling previous request. Action aborted."));
        }
        m_spawnerStateQueue.push(SpawnerState::Despawning);
        DespawnEntitiesById(idsToDelete);
        AZ::TickBus::Handler::BusConnect();

        return AZ::Success();
    }

    Result GeoJSONSpawnerComponent::DeleteAll()
    {
        if (m_spawnerState != SpawnerState::Idle || !m_spawnerStateQueue.empty())
        {
            return AZ::Failure(AZStd::string("Spawner is handling previous request. Action aborted."));
        }
        m_spawnerStateQueue.push(SpawnerState::Despawning);
        DespawnAllEntities();
        AZ::TickBus::Handler::BusConnect();

        return AZ::Success();
    }

    GetIdsResult GeoJSONSpawnerComponent::GetIds() const
    {
        AZStd::string result{ "" };
        for (const auto& pair : m_spawnableTickets)
        {
            const int id = pair.first;
            const size_t objCount = pair.second.size();

            result += AZStd::string::format("[GroupID: %d: %zu Ticket(s)], ", id, objCount);
        }

        return AZ::Success(result);
    }

    void GeoJSONSpawnerComponent::OnTerrainDataCreateEnd()
    {
        if (m_terrainCreatedOnlyOnce)
        {
            return;
        }

        AZ::TickBus::QueueFunction(
            [this]()
            {
                SpawnEntities();
            });

        // Init only once, even if level have multiple terrains
        m_terrainCreatedOnlyOnce = true;
    }

    void GeoJSONSpawnerComponent::OnTerrainDataDestroyBegin()
    {
        m_terrainCreatedOnlyOnce = false;
        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusDisconnect();
    }

    void GeoJSONSpawnerComponent::FillGroupIdToTicketIdMap()
    {
        // m_spawnableTicketsIds.clear();
        for (const auto& pair : m_spawnableTickets)
        {
            m_spawnableTicketsIds[pair.first] = {};
            for (const auto& ticket : pair.second)
            {
                m_spawnableTicketsIds[pair.first].insert(ticket.GetId());
                m_ticketsToDespawn++;
            }
        }
    }

    void GeoJSONSpawnerComponent::FillGroupIdToTicketIdMap(const AZStd::unordered_set<int>& groupIds)
    {
        m_spawnableTicketsIds.clear();
        for (const auto id : groupIds)
        {
            if (auto it = m_spawnableTickets.find(id); it == m_spawnableTickets.end())
            {
                continue;
            }
            m_spawnableTicketsIds[id] = {};
            for (const auto& ticket : m_spawnableTickets[id])
            {
                m_spawnableTicketsIds[id].insert(ticket.GetId());
                m_ticketsToDespawn++;
            }
        }
    }

    unsigned int GeoJSONSpawnerComponent::CountTicketsToSpawn(
        const AZStd::unordered_map<int, AZStd::vector<GeoJSONUtils::TicketToSpawnPair>>& ticketsToSpawn) const
    {
        unsigned int count = 0;
        for (const auto& tickets : ticketsToSpawn)
        {
            count += tickets.second.size();
        }
        return count;
    }

    void GeoJSONSpawnerComponent::Despawn(AzFramework::EntitySpawnTicket& ticketToDespawn)
    {
        if (!ticketToDespawn.IsValid())
        {
            m_despawnStatus |= GeoJSONUtils::SpawnDespawnStatus::Warning;
            return;
        }

        GeoJSONUtils::DespawnEntity(
            ticketToDespawn,
            [this](auto id)
            {
                for (auto& pair : m_spawnableTicketsIds)
                {
                    if (auto it = pair.second.find(id); it != pair.second.end())
                    {
                        pair.second.erase(it);
                    }
                    if (pair.second.empty())
                    {
                        m_spawnableTickets.erase(pair.first);
                    }
                }
                m_ticketsToDespawn--;
            });
    }

    void GeoJSONSpawnerComponent::DespawnAllEntities()
    {
        // Call GeoJSONSpawner EBus notification - Despawn Begin
        GeoJSONSpawnerNotificationBus::Broadcast(&GeoJSONSpawnerInterface::OnEntitiesDespawnBegin);

        // Reset previous spawn status
        m_despawnStatus = InitSpawnDespawnStatus();

        FillGroupIdToTicketIdMap();

        auto copyOfTickets = m_spawnableTickets;

        for (auto& pair : m_spawnableTickets)
        {
            for (auto& ticket : pair.second)
            {
                Despawn(ticket);
            }
        }

        if (m_ticketsToDespawn > 0)
        {
            m_despawnStatus |= GeoJSONUtils::SpawnDespawnStatus::Warning;
        }

        // Call GeoJSONSpawner EBus notification - Despawn Finished
        GeoJSONSpawnerNotificationBus::Broadcast(&GeoJSONSpawnerInterface::OnEntitiesDespawnFinished, copyOfTickets, m_despawnStatus);
    }

    void GeoJSONSpawnerComponent::DespawnEntitiesById(const GeoJSONUtils::Ids& ids)
    {
        // Call GeoJSONSpawner EBus notification - Despawn Begin
        GeoJSONSpawnerNotificationBus::Broadcast(&GeoJSONSpawnerInterface::OnEntitiesDespawnBegin);

        // Reset previous spawn status
        m_despawnStatus = InitSpawnDespawnStatus();
        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> copyOfTickets;

        FillGroupIdToTicketIdMap(ids);

        for (const auto idToDespawn : ids)
        {
            auto it = m_spawnableTickets.find(idToDespawn);
            if (it == m_spawnableTickets.end())
            {
                AZ_Error(
                    "GeoJSONSpawnerComponent", false, "Cannot delete entities. Entities group with ID: %d does not exist.", idToDespawn);

                m_despawnStatus |= GeoJSONUtils::SpawnDespawnStatus::Warning;
                continue;
            }

            copyOfTickets[idToDespawn] = it->second; // Copy before modification

            for (auto& ticket : m_spawnableTickets[idToDespawn])
            {
                Despawn(ticket);
            }
        }

        if (copyOfTickets.empty())
        {
            m_despawnStatus |= GeoJSONUtils::SpawnDespawnStatus::Warning | GeoJSONUtils::SpawnDespawnStatus::Fail;
        }

        // Call GeoJSONSpawner EBus notification - Despawn Finished
        GeoJSONSpawnerNotificationBus::Broadcast(&GeoJSONSpawnerInterface::OnEntitiesDespawnFinished, copyOfTickets, m_despawnStatus);
    }

    GeoJSONUtils::SpawnDespawnStatus GeoJSONSpawnerComponent::InitSpawnDespawnStatus() const
    {
        // When called - tickets cannot be empty
        return m_spawnableTicketsIds.empty() || m_spawnableTickets.empty() ? GeoJSONUtils::SpawnDespawnStatus::Fail
                                                                           : GeoJSONUtils::SpawnDespawnStatus::Success;
    }
} // namespace GeoJSONSpawner
