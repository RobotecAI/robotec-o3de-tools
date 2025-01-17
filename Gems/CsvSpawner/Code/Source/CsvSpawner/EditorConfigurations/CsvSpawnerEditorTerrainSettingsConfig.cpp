/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "CsvSpawnerEditorTerrainSettingsConfig.h"

#include "AzCore/Serialization/EditContext.h"
#include "AzCore/Serialization/EditContextConstants.inl"
#include "AzCore/Serialization/SerializeContext.h"
#include "CsvSpawner/CsvSpawnerUtils.h"

namespace CsvSpawner
{
    void CsvSpawnerEditorTerrainSettingsConfig::Reflect(AZ::ReflectContext* context)
    {
        auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<CsvSpawnerEditorTerrainSettingsConfig>()
                ->Version(2)
                ->Field("SpawnOnComponentActivated", &CsvSpawnerEditorTerrainSettingsConfig::m_spawnOnComponentActivated)
                ->Field("SpawnOnTerrainUpdate", &CsvSpawnerEditorTerrainSettingsConfig::m_spawnOnTerrainUpdate)
                ->Field("TerrainDataChangedMask", &CsvSpawnerEditorTerrainSettingsConfig::m_terrainMasksToIgnore);
        }

        auto* editContext = serializeContext->GetEditContext();
        if (editContext)
        {
            editContext
                ->Class<CsvSpawnerEditorTerrainSettingsConfig>(
                    "CsvSpawnerEditorTerrainSettingsConfig", "CsvSpawnerEditorTerrainSettingsConfig")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "In Editor Spawn Settings")
                ->Attribute(AZ::Edit::Attributes::Category, "In Editor Spawn Settings")
                ->DataElement(
                    AZ::Edit::UIHandlers::Default,
                    &CsvSpawnerEditorTerrainSettingsConfig::m_spawnOnComponentActivated,
                    "Spawn On Editor Activate",
                    "Spawns entities when editor component is being activated.")
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorTerrainSettingsConfig::RefreshUI)
                ->DataElement(
                    AZ::Edit::UIHandlers::Default,
                    &CsvSpawnerEditorTerrainSettingsConfig::m_spawnOnTerrainUpdate,
                    "Spawn On Terrain Update",
                    "Should respawn entities on any Terrain config and transform change.")
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorTerrainSettingsConfig::SpawnOnTerrainUpdateTriggered)
                ->Attribute(AZ::Edit::Attributes::Visibility, &CsvSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain)
                ->DataElement(
                    AZ::Edit::UIHandlers::ComboBox,
                    &CsvSpawnerEditorTerrainSettingsConfig::m_terrainMasksToIgnore,
                    "Terrain Flags To Ignore",
                    "Flags to ignore on the terrain update data performed.")
                ->Attribute(AZ::Edit::Attributes::ReadOnly, &CsvSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateDisabled)
                ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorTerrainSettingsConfig::OnTerrainFlagsChanged)
                ->Attribute(AZ::Edit::Attributes::Visibility, &CsvSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain)
                ->Attribute(AZ::Edit::Attributes::ComboBoxEditable, &CsvSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateEnabled)
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

    AZ::u32 CsvSpawnerEditorTerrainSettingsConfig::SetPropertyVisibilityByTerrain() const
    {
        return CsvSpawnerUtils::IsTerrainAvailable() ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 CsvSpawnerEditorTerrainSettingsConfig::SpawnOnTerrainUpdateTriggered()
    {
        if (IsSpawnOnTerrainUpdateDisabled())
        {
            m_terrainMasksToIgnore = AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::All;
        }

        return RefreshUI();
    }

    AZ::Crc32 CsvSpawnerEditorTerrainSettingsConfig::OnTerrainFlagsChanged()
    {
        if (m_terrainMasksToIgnore == AzFramework::Terrain::TerrainDataNotifications::TerrainDataChangedMask::All)
        {
            m_spawnOnTerrainUpdate = false;
        }

        return RefreshUI();
    }

    AZ::Crc32 CsvSpawnerEditorTerrainSettingsConfig::RefreshUI()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    bool CsvSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateDisabled() const
    {
        return !m_spawnOnTerrainUpdate;
    }

    bool CsvSpawnerEditorTerrainSettingsConfig::IsSpawnOnTerrainUpdateEnabled() const
    {
        return m_spawnOnTerrainUpdate;
    }
} // namespace CsvSpawner