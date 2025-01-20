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

#include "AzCore/RTTI/ReflectContext.h"
#include "AzFramework/Terrain/TerrainDataRequestBus.h"
#include "GeoJSONSpawner/GeoJSONSpawnerTypeIds.h"

namespace GeoJSONSpawner
{
    //! Terrain Settings Configuration for Editor Component.
    //! This config lets user decide what behaviour should be applied when the Terrain is applicable in current Level.
    class GeoJSONSpawnerEditorTerrainSettingsConfig
    {
    public:
        AZ_RTTI(GeoJSONSpawnerEditorTerrainSettingsConfig, GeoJSONSpawnerEditorTerrainSettingsConfigTypeId)

        GeoJSONSpawnerEditorTerrainSettingsConfig() = default;
        virtual ~GeoJSONSpawnerEditorTerrainSettingsConfig() = default;

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

} // namespace GeoJSONSpawner
