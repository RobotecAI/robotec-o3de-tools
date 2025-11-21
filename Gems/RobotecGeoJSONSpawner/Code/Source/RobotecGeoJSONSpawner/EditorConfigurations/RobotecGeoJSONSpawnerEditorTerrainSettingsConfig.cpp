/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerUtils.h"
#include "RobotecGeoJSONSpawnerEditorTerrainSettingsConfig.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecGeoJSONSpawner
{
    void RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::Reflect(AZ::ReflectContext* context)
    {
        auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<RobotecGeoJSONSpawnerEditorTerrainSettingsConfig>()
                ->Version(0)
                ->Field("SpawnOnComponentActivated", &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnComponentActivated)
                ->Field("SpawnOnTerrainUpdate", &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnTerrainUpdate)
                ->Field("TerrainDataChangedMask", &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::m_terrainMasksToIgnore);
        }

        auto* editContext = serializeContext->GetEditContext();
        if (editContext)
        {
            editContext
                ->Class<RobotecGeoJSONSpawnerEditorTerrainSettingsConfig>(
                    "RobotecGeoJSONSpawnerEditorTerrainSettingsConfig", "RobotecGeoJSONSpawnerEditorTerrainSettingsConfig")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "In Editor Spawn Settings")
                ->Attribute(AZ::Edit::Attributes::Category, "In Editor Spawn Settings")
                ->DataElement(
                    AZ::Edit::UIHandlers::Default,
                    &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnComponentActivated,
                    "Spawn On Editor Activate",
                    "Spawns entities when editor component is being activated.")
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::RefreshUI)
                ->DataElement(
                    AZ::Edit::UIHandlers::Default,
                    &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnTerrainUpdate,
                    "Spawn On Terrain Update",
                    "Should respawn entities on any Terrain config and transform change.")
                ->Attribute(
                    AZ::Edit::Attributes::ChangeNotify, &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::SpawnOnTerrainUpdateTriggered)
                ->Attribute(
                    AZ::Edit::Attributes::Visibility, &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain)
                ->DataElement(
                    AZ::Edit::UIHandlers::ComboBox,
                    &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::m_terrainMasksToIgnore,
                    "Terrain Flags To Ignore",
                    "Flags to ignore on the terrain update data performed.")
                ->Attribute(
                    AZ::Edit::Attributes::ReadOnly, &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateDisabled)
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::OnTerrainFlagsChanged)
                ->Attribute(
                    AZ::Edit::Attributes::Visibility, &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain)
                ->Attribute(
                    AZ::Edit::Attributes::ComboBoxEditable,
                    &RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateEnabled)
                ->Attribute(
                    AZ::Edit::Attributes::EnumValues,
                    AZStd::vector<AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>>{
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::None, "None"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::All, "All"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings, "Settings"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::HeightData, "Height Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData, "Color Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::SurfaceData, "Surface Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::HeightData,
                            "Settings + Height Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData,
                            "Settings + Color Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::SurfaceData,
                            "Settings + Surface Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::HeightData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData,
                            "Height Data + Color Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::HeightData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::SurfaceData,
                            "Height Data + Surface Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::SurfaceData,
                            "Color Data + Surface Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::HeightData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData,
                            "Settings + Height Data + Color Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::HeightData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::SurfaceData,
                            "Settings + Height Data + Surface Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::Settings |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::SurfaceData,
                            "Settings + Color Data + Surface Data"),
                        AZ::Edit::EnumConstant<AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask>(
                            AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::HeightData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::ColorData |
                                AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::SurfaceData,
                            "Height Data + Color Data + Surface Data"),
                    });
        }
    }

    AZ::u32 RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain() const
    {
        return GeoJSONUtils::IsTerrainAvailable() ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::SpawnOnTerrainUpdateTriggered()
    {
        return RefreshUI();
    }

    AZ::Crc32 RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::OnTerrainFlagsChanged()
    {
        if (m_terrainMasksToIgnore == AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::All)
        {
            m_spawnOnTerrainUpdate = false;
        }

        return RefreshUI();
    }

    AZ::Crc32 RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::RefreshUI()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    bool RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateDisabled() const
    {
        return !m_spawnOnTerrainUpdate;
    }

    bool RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateEnabled() const
    {
        return m_spawnOnTerrainUpdate;
    }
} // namespace RobotecGeoJSONSpawner
