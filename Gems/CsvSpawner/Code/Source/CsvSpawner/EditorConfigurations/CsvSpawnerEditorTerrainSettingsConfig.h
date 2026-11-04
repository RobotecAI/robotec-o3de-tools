/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "CsvSpawner/CsvSpawnerTypeIds.h"
#include <AzFramework/Terrain/TerrainDataRequestBus.h>

namespace CsvSpawner
{

    //! Terrain Settings Configuration for Editor Component.
    //! This config lets user decide what behaviour should be applied when the Terrain is applicable in current Level.
    class CsvSpawnerEditorTerrainSettingsConfig
    {
    public:
        AZ_RTTI(CsvSpawnerEditorTerrainSettingsConfig, CsvSpawnerEditorTerrainSettingsConfigTypeId)

        CsvSpawnerEditorTerrainSettingsConfig() = default;
        virtual ~CsvSpawnerEditorTerrainSettingsConfig() = default;

        static void Reflect(AZ::ReflectContext* context);

        //! Whether entities should be spawned if editor component is activated.
        bool m_spawnOnComponentActivated{ true };

        //! Prevent multiple terrains to init spawn. @returns True if spawned once, false otherwise.
        bool m_flagSpawnEntitiesOnStartOnce{ false };

        //! Whether Terrain settings or position is updated, and if should spawned entities follow up the changes.
        bool m_spawnOnTerrainUpdate{ false };

        //! Masks to be ignored while updating Terrain.
        AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask m_terrainMasksToIgnore{
            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings |
            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData
        };

    private:
        AZ::u32 SetPropertyVisibilityByTerrain() const;

        AZ::Crc32 SpawnOnTerrainUpdateTriggered();

        AZ::Crc32 OnTerrainFlagsChanged();

        AZ::Crc32 RefreshUI();

        // Helper functions for UI Notify (need both, since cannot negate them in Reflect)
        [[nodiscard]] bool IsSpawnOnTerrainUpdateDisabled() const;
        [[nodiscard]] bool IsSpawnOnTerrainUpdateEnabled() const;
    };

} // namespace CsvSpawner
