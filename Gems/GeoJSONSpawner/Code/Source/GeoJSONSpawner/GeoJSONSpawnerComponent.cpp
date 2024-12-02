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

#include "AzCore/std/smart_ptr/make_shared.h"

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
        AZ::TickBus::Handler::BusConnect();
        GeoJSONSpawnerRequestBus::Handler::BusConnect(GetEntityId());
    }

    void GeoJSONSpawnerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
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

    void GeoJSONSpawnerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_isInSpawningProcess)
        {
            if (m_spawnableTickets.empty())
            {
                m_isInSpawningProcess = false;
                Spawn(m_cachedGeoJson);
            }
        }

        if (m_isInModifyProcess)
        {
            if (m_ticketsToDespawn == 0)
            {
                m_isInModifyProcess = false;
                SpawnModifiedEntities(m_cachedGeoJson);
            }
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
        if (!m_spawnableTickets.empty() && !m_isInSpawningProcess)
        {
            m_isInSpawningProcess = true;
            m_cachedGeoJson = rawJsonString;
            DespawnAllEntities();
        }
        else
        {
            const auto result = GeoJSONUtils::ParseJSONFromRawString(rawJsonString);
            m_spawnableEntityInfo = GeoJSONUtils::GetSpawnableEntitiesFromGeometryObjectVector(result, m_spawnableAssetConfigurations);

            m_spawnableTickets = GeoJSONUtils::SpawnEntities(
                m_spawnableEntityInfo, m_spawnableAssetConfigurations, m_defaultSeed, AzPhysics::DefaultPhysicsSceneName, GetEntityId());
        }
    }

    void GeoJSONSpawnerComponent::SpawnModifiedEntities(const AZStd::string& rawJsonString)
    {
        const auto result = GeoJSONUtils::ParseJSONFromRawString(rawJsonString);
        auto entityInfos = GeoJSONUtils::GetSpawnableEntitiesFromGeometryObjectVector(result, m_spawnableAssetConfigurations);

        const auto spawnableTickets = GeoJSONUtils::SpawnEntities(
            entityInfos, m_spawnableAssetConfigurations, m_defaultSeed, AzPhysics::DefaultPhysicsSceneName, GetEntityId());

        for (const auto& pair : spawnableTickets)
        {
            m_spawnableTickets[pair.first] = AZStd::move(pair.second);
        }
    }

    void GeoJSONSpawnerComponent::Modify(const AZStd::string& rawJsonString)
    {
        const auto groupIdsToModify = GeoJSONUtils::ExtractIdsFromRawString(rawJsonString);

        for (const auto& groupId : groupIdsToModify)
        {
            if (m_spawnableTickets.find(groupId) == m_spawnableTickets.end())
            {
                AZ_Error(
                    "GeoJSONSpawnerComponent",
                    false,
                    "Cannot modify entities with ID: %d. Such group does not exist. Action aborted.",
                    groupId);
                return;
            }
        }

        m_isInModifyProcess = true;
        m_cachedGeoJson = rawJsonString;
        DespawnEntitiesById(groupIdsToModify);
    }

    void GeoJSONSpawnerComponent::DeleteById(const AZStd::string& rawJsonString)
    {
        const auto groupIdsToRemove = GeoJSONUtils::ExtractIdsFromRawString(rawJsonString);
        DespawnEntitiesById(groupIdsToRemove);
    }

    void GeoJSONSpawnerComponent::DeleteAll()
    {
        DespawnAllEntities();
    }

    AZStd::string GeoJSONSpawnerComponent::GetIds() const
    {
        AZStd::string result{ "" };
        for (const auto& pair : m_spawnableTickets)
        {
            const int id = pair.first;
            const size_t objCount = pair.second.size();

            result += AZStd::string::format("[GroupID: %d: %zu Ticket(s)],", id, objCount);
        }

        return result;
    }

    void GeoJSONSpawnerComponent::FillGroupIdToTicketIdMap()
    {
        m_spawnableTicketsIds.clear();
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

    void GeoJSONSpawnerComponent::Despawn(AzFramework::EntitySpawnTicket& ticketToDespawn)
    {
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
        FillGroupIdToTicketIdMap();

        for (auto& pair : m_spawnableTickets)
        {
            for (auto& ticket : pair.second)
            {
                Despawn(ticket);
            }
        }
    }

    void GeoJSONSpawnerComponent::DespawnEntitiesById(const GeoJSONUtils::Ids& ids)
    {
        FillGroupIdToTicketIdMap(ids);

        for (const auto idToDespawn : ids)
        {
            if (auto it = m_spawnableTickets.find(idToDespawn); it == m_spawnableTickets.end())
            {
                AZ_Error(
                    "GeoJSONSpawnerComponent", false, "Cannot delete entities. Entities group with ID: %d does not exist.", idToDespawn);
                continue;
            }

            for (auto& ticket : m_spawnableTickets[idToDespawn])
            {
                Despawn(ticket);
            }
        }
    }

} // namespace GeoJSONSpawner
