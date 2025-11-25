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

#include "EditorConfigurations/RobotecGeoJSONSpawnerEditorTerrainSettingsConfig.h"
#include "RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerTypeIds.h"
#include "RobotecGeoJSONSpawnerUtils.h"

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzFramework/Terrain/TerrainDataRequestBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace RobotecGeoJSONSpawner
{
    //! Editor component for the RobotecGeoJSONSpawner component.
    //! This component is used to spawn using GeoJSON in the editor.
    //! It loads a GeoJSON file that contains WGS84 coordinates to spawn entities.
    //! It also allows the user to set configuration parameters for the GeoJSON to spawn (@see
    //! RobotecGeoJSONSpawnerUtils::GeoJSONSpawnableAssetConfiguration).
    class RobotecGeoJSONSpawnerEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , protected AzFramework::ViewportDebugDisplayEventBus::Handler
        , protected AzFramework::Terrain::TerrainDataNotificationBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(RobotecGeoJSONSpawnerEditorComponent, RobotecGeoJSONSpawnerEditorComponentTypeId);

        RobotecGeoJSONSpawnerEditorComponent() = default;
        ~RobotecGeoJSONSpawnerEditorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase interface overrides
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        // AzFramework::Terrain::TerrainDataNotificationBus::Handler overrides
        void OnTerrainDataChanged([[maybe_unused]] const AZ::Aabb& dirtyRegion, TerrainDataChangedMask dataChangedMask) override;

    private:
        // EntityDebugDisplayEventBus::Handler overrides
        void DisplayViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        void SpawnEntities();

        void OnSpawnButton();
        void OnShowLabelsChanged();

        AZStd::vector<GeoJSONUtils::GeoJSONSpawnableAssetConfiguration> m_spawnableAssetConfigurations;
        AZ::Data::AssetId m_geoJsonAssetId;
        AZ::u64 m_defaultSeed{ 0 };
        bool m_showLabels{ false };

        AZStd::vector<GeoJSONUtils::GeoJSONSpawnableEntityInfo> m_spawnableEntityInfo;
        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> m_spawnedTicketsGroups;

        RobotecGeoJSONSpawnerEditorTerrainSettingsConfig
            m_terrainSettingsConfig; //!< Terrain Editor Settings Configuration for RobotecGeoJSONSpawner
    };

} // namespace RobotecGeoJSONSpawner
