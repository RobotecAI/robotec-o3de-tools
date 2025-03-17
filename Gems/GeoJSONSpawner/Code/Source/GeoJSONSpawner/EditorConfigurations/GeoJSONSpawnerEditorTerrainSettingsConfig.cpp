/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerEditorTerrainSettingsConfig.h"
#include "GeoJSONSpawner/GeoJSONSpawnerUtils.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace GeoJSONSpawner
{
    void GeoJSONSpawnerEditorTerrainSettingsConfig::Reflect(AZ::ReflectContext* context)
    {
        auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<GeoJSONSpawnerEditorTerrainSettingsConfig>()
                ->Version(0)
                ->Field("SpawnOnComponentActivated", &GeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnComponentActivated)
                ->Field("SpawnOnTerrainUpdate", &GeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnTerrainUpdate)
                ->Field("TerrainDataChangedMask", &GeoJSONSpawnerEditorTerrainSettingsConfig::m_terrainMasksToIgnore);
        }

        auto* editContext = serializeContext->GetEditContext();
        if (editContext)
        {
            editContext
                ->Class<GeoJSONSpawnerEditorTerrainSettingsConfig>(
                    "GeoJSONSpawnerEditorTerrainSettingsConfig", "GeoJSONSpawnerEditorTerrainSettingsConfig")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "In Editor Spawn Settings")
                ->Attribute(AZ::Edit::Attributes::Category, "In Editor Spawn Settings")
                ->DataElement(
                    AZ::Edit::UIHandlers::Default,
                    &GeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnComponentActivated,
                    "Spawn On Editor Activate",
                    "Spawns entities when editor component is being activated.")
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GeoJSONSpawnerEditorTerrainSettingsConfig::RefreshUI)
                ->DataElement(
                    AZ::Edit::UIHandlers::Default,
                    &GeoJSONSpawnerEditorTerrainSettingsConfig::m_spawnOnTerrainUpdate,
                    "Spawn On Terrain Update",
                    "Should respawn entities on any Terrain config and transform change.")
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GeoJSONSpawnerEditorTerrainSettingsConfig::SpawnOnTerrainUpdateTriggered)
                ->Attribute(AZ::Edit::Attributes::Visibility, &GeoJSONSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain)
                ->DataElement(
                    AZ::Edit::UIHandlers::ComboBox,
                    &GeoJSONSpawnerEditorTerrainSettingsConfig::m_terrainMasksToIgnore,
                    "Terrain Flags To Ignore",
                    "Flags to ignore on the terrain update data performed.")
                ->Attribute(AZ::Edit::Attributes::ReadOnly, &GeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateDisabled)
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GeoJSONSpawnerEditorTerrainSettingsConfig::OnTerrainFlagsChanged)
                ->Attribute(AZ::Edit::Attributes::Visibility, &GeoJSONSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain)
                ->Attribute(
                    AZ::Edit::Attributes::ComboBoxEditable, &GeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateEnabled)
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

    AZ::u32 GeoJSONSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain() const
    {
        return GeoJSONUtils::IsTerrainAvailable() ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 GeoJSONSpawnerEditorTerrainSettingsConfig::SpawnOnTerrainUpdateTriggered()
    {
        return RefreshUI();
    }

    AZ::Crc32 GeoJSONSpawnerEditorTerrainSettingsConfig::OnTerrainFlagsChanged()
    {
        if (m_terrainMasksToIgnore == AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::All)
        {
            m_spawnOnTerrainUpdate = false;
        }

        return RefreshUI();
    }

    AZ::Crc32 GeoJSONSpawnerEditorTerrainSettingsConfig::RefreshUI()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    bool GeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateDisabled() const
    {
        return !m_spawnOnTerrainUpdate;
    }

    bool GeoJSONSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateEnabled() const
    {
        return m_spawnOnTerrainUpdate;
    }
} // namespace GeoJSONSpawner