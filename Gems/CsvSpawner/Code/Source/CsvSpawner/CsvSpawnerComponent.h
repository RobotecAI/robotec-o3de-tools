/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "AzFramework/Terrain/TerrainDataRequestBus.h"
#include "CsvSpawnerUtils.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace CsvSpawner
{
    using namespace CsvSpawnerUtils;

    //! Game component that spawns entities from a CSV file.
    //! This component is used to spawn trees, on the first tick.
    //! In contrast to the CsvSpawnerEditorComponent, this component's reflection contains all the data it needs to spawn trees ( it is not
    //! dependent on a CSV file).
    class CsvSpawnerComponent
        : public AZ::Component
        , protected AzFramework::Terrain::TerrainDataNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(CsvSpawnerComponent, CsvSpawnerComponentTypeId);

        CsvSpawnerComponent() = default;

        CsvSpawnerComponent(
            const AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration>& spawnableAssetConfiguration,
            const AZStd::vector<CsvSpawnableEntityInfo>& spawnableEntityInfo,
            AZ::u64 defaultSeed);

        ~CsvSpawnerComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

        // AzFramework::Terrain::TerrainDataNotificationBus::Handler overrides
        void OnTerrainDataCreateEnd() override;
        void OnTerrainDataDestroyBegin() override;

    private:
        AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration> m_spawnableAssetConfigurations; //!< List of assets to spawn
        AZStd::vector<CsvSpawnableEntityInfo> m_spawnableEntityInfo; //!< List of entities to spawn
        AZ::u64 m_defaultSeed{ 0 }; //!< Default seed value

        AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> m_spawnedTickets;

        // Terrain notify
        bool m_terrainCreatedOnlyOnce{ false }; //!< Is terrain fully generated once
        [[nodiscard]] static bool IsTerrainAvailable(); //!< @returns True if current level has terrain, otherwise false.
    };
} // namespace CsvSpawner
