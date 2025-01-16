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

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace CsvSpawner
{

    //! Editor component for the CsvSpawner component.
    //! This component is used to spawn Csvs in the editor.
    //! It loads a CSV file that contains location of Csvs to spawn.
    //! It also allows the user to set configuration parameters for the Csvs to spawn (@see CsvSpawnerUtils::SpawnableAssetConfiguration).
    class CsvSpawnerEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , protected AzFramework::ViewportDebugDisplayEventBus::Handler
        , protected AzFramework::Terrain::TerrainDataNotificationBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(CsvSpawnerEditorComponent, CsvSpawnerEditorComponentTypeId);
        CsvSpawnerEditorComponent() = default;
        ~CsvSpawnerEditorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        // AzFramework::Terrain::TerrainDataNotificationBus interface overrides ...
        void OnTerrainDataChanged(const AZ::Aabb& dirtyRegion, TerrainDataChangedMask dataChangedMask) override;

    private:
        // EntityDebugDisplayEventBus::Handler overrides
        void DisplayViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        void OnSpawnButton();

        void OnShowLabelsChanged();

        void SpawnEntities();

        AZStd::vector<CsvSpawnerUtils::CsvSpawnableAssetConfiguration>
            m_spawnableAssetConfigurations; //!< List of spawnable "types" (e.g. pineCsv, oakTre, mapleCsv, etc.)
        AZStd::vector<CsvSpawnerUtils::CsvSpawnableEntityInfo> m_spawnableEntityInfo; //!< List of spawnable "entities"
        AZ::Data::AssetId m_csvAssetId; //!< Asset ID of the CSV file
        AZ::u64 m_defaultSeed{ 0 }; //!< Default seed value
        bool m_showLabels{ true }; //!< Whether to show labels or not in Editorullq

        AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> m_spawnedTickets; //!< Tickets for editor-time spawned entities
        int m_numberOfEntries{ 0 }; //!< Number of entries in the csv file

        bool m_spawnOnComponentActivated{ true }; //!< Whether entities should be spawned if editor component is activated.
        bool m_flagSpawnEntitiesOnStartOnce{ false }; //!< @returns True if spawned once, false otherwise.

        //! Whether Terrain settings or position is updated, and if should spawned entities follow up the changes.
        bool m_spawnOnTerrainUpdate{ false };
        AZ::u32 SetPropertyVisibilityByTerrain() const;
        TerrainDataChangedMask m_terrainDataChangedMask{ TerrainDataChangedMask::Settings | TerrainDataChangedMask::ColorData };
        void OnTerrainFlagsChanged();
    };
} // namespace CsvSpawner
