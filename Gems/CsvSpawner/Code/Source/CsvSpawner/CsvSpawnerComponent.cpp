/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CsvSpawnerComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

namespace CsvSpawner
{
    using namespace CsvSpawnerUtils;

    void CsvSpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        CsvSpawnerUtils::CsvSpawnableAssetConfiguration::Reflect(context);
        CsvSpawnerUtils::CsvSpawnableEntityInfo::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<CsvSpawnerComponent, AZ::Component>()
                ->Version(1)
                ->Field("SpawnableAssetConfigurations", &CsvSpawnerComponent::m_spawnableAssetConfigurations)
                ->Field("SpawnableEntityInfo", &CsvSpawnerComponent::m_spawnableEntityInfo)
                ->Field("DefautlSeed", &CsvSpawnerComponent::m_defaultSeed);
        }
    }

    CsvSpawnerComponent::CsvSpawnerComponent(
        const AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration>& spawnableAssetConfiguration,
        const AZStd::vector<CsvSpawnableEntityInfo>& spawnableEntityInfo,
        AZ::u64 defaultSeed)
        : m_spawnableAssetConfigurations(spawnableAssetConfiguration)
        , m_spawnableEntityInfo(spawnableEntityInfo)
        , m_defaultSeed(defaultSeed)
    {
    }

    void CsvSpawnerComponent::Activate()
    {
        if (IsTerrainAvailable())
        {
            AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusConnect();
        }
        else
        {
            OnTerrainDataCreateEnd();
        }
    }

    void CsvSpawnerComponent::Deactivate()
    {
        OnTerrainDataDestroyBegin();
    }

    void CsvSpawnerComponent::OnTerrainDataCreateEnd()
    {
        if (m_terrainCreatedOnlyOnce)
        {
            return;
        }

        AZ::TickBus::QueueFunction(
            [this]()
            {
                m_spawnedTickets = CsvSpawnerUtils::SpawnEntities(
                    m_spawnableEntityInfo,
                    m_spawnableAssetConfigurations,
                    m_defaultSeed,
                    AzPhysics::DefaultPhysicsSceneName,
                    this->GetEntityId());
            });

        // Init only once, even if level have multiple terrains
        m_terrainCreatedOnlyOnce = true;
    }

    void CsvSpawnerComponent::OnTerrainDataDestroyBegin()
    {
        m_terrainCreatedOnlyOnce = false;
        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusDisconnect();
    }
} // namespace CsvSpawner
