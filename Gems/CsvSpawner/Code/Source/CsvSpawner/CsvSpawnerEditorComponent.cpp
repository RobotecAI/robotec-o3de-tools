/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "CsvSpawnerEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include "AzFramework/Physics/PhysicsScene.h"
#include "CsvSpawnerComponent.h"
#include "CsvSpawnerCsvParser.h"
#include "CsvSpawnerUtils.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Viewport/ViewportMessages.h>

namespace CsvSpawner
{
    void CsvSpawnerEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            // Reflect the enum
            serializeContext->Enum<TerrainDataChangedMask>()
                ->Value("None", TerrainDataChangedMask::None)
                ->Value("Settings", TerrainDataChangedMask::Settings)
                ->Value("HeightData", TerrainDataChangedMask::HeightData)
                ->Value("ColorData", TerrainDataChangedMask::ColorData)
                ->Value("SurfaceData", TerrainDataChangedMask::SurfaceData)
                ->Value("All", TerrainDataChangedMask::All);

            serializeContext->Class<CsvSpawnerEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("CsvAssetId", &CsvSpawnerEditorComponent::m_csvAssetId)
                ->Field("NumberOfEntries", &CsvSpawnerEditorComponent::m_numberOfEntries)
                ->Field("SpawnableAssetConfigurations", &CsvSpawnerEditorComponent::m_spawnableAssetConfigurations)
                ->Field("DefaultSeed", &CsvSpawnerEditorComponent::m_defaultSeed)
                ->Field("ShowLabels", &CsvSpawnerEditorComponent::m_showLabels)
                ->Field("SpawnOnComponentActivated", &CsvSpawnerEditorComponent::m_spawnOnComponentActivated)
                ->Field("SpawnOnTerrainUpdate", &CsvSpawnerEditorComponent::m_spawnOnTerrainUpdate)
                ->Field("TerrainDataChangedMask", &CsvSpawnerEditorComponent::m_terrainDataChangedMask);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<CsvSpawnerEditorComponent>("CsvSpawnerEditorComponent", "CsvSpawnerEditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "CsvSpawnerEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "CsvSpawner")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnerEditorComponent::m_spawnableAssetConfigurations,
                        "Asset Config",
                        "Asset configuration")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_csvAssetId, "CSV Asset", "CSV asset")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Reload Csv", "Reload Csv")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Spawn")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorComponent::OnSpawnButton)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_numberOfEntries, "Number of entries", "")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_defaultSeed, "Default seed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_showLabels, "Show labels in Editor", "")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorComponent::OnShowLabelsChanged)
                    ->ClassElement(AZ::Edit::ClassElements::Group, "In Editor Spawn Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnerEditorComponent::m_spawnOnComponentActivated,
                        "Spawn On Editor Activate",
                        "Spawns entities when editor component is being activated.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorComponent::RefreshUI)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnerEditorComponent::m_spawnOnTerrainUpdate,
                        "Spawn On Terrain Update",
                        "Should respawn entities on any Terrain config and transform change.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorComponent::SpawnOnTerrainUpdateTriggered)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &CsvSpawnerEditorComponent::SetPropertyVisibilityByTerrain)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &CsvSpawnerEditorComponent::m_terrainDataChangedMask,
                        "Terrain Flags To Ignore",
                        "Flags to ignore on the terrain update data performed.")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, &CsvSpawnerEditorComponent::IsSpawnOnTerrainUpdateDisabled)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorComponent::OnTerrainFlagsChanged)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &CsvSpawnerEditorComponent::SetPropertyVisibilityByTerrain)
                    ->Attribute(AZ::Edit::Attributes::ComboBoxEditable, &CsvSpawnerEditorComponent::IsSpawnOnTerrainUpdateEnabled)
                    ->Attribute(
                        AZ::Edit::Attributes::EnumValues,
                        AZStd::vector<AZ::Edit::EnumConstant<TerrainDataChangedMask>>{
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(TerrainDataChangedMask::None, "None"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(TerrainDataChangedMask::All, "All"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(TerrainDataChangedMask::Settings, "Settings"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(TerrainDataChangedMask::HeightData, "Height Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(TerrainDataChangedMask::ColorData, "Color Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(TerrainDataChangedMask::SurfaceData, "Surface Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::Settings | TerrainDataChangedMask::HeightData, "Settings + Height Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::Settings | TerrainDataChangedMask::ColorData, "Settings + Color Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::Settings | TerrainDataChangedMask::SurfaceData, "Settings + Surface Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::HeightData | TerrainDataChangedMask::ColorData, "Height Data + Color Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::HeightData | TerrainDataChangedMask::SurfaceData, "Height Data + Surface Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::ColorData | TerrainDataChangedMask::SurfaceData, "Color Data + Surface Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::Settings | TerrainDataChangedMask::HeightData | TerrainDataChangedMask::ColorData,
                                "Settings + Height Data + Color Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::Settings | TerrainDataChangedMask::HeightData | TerrainDataChangedMask::SurfaceData,
                                "Settings + Height Data + Surface Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::Settings | TerrainDataChangedMask::ColorData | TerrainDataChangedMask::SurfaceData,
                                "Settings + Color Data + Surface Data"),
                            AZ::Edit::EnumConstant<TerrainDataChangedMask>(
                                TerrainDataChangedMask::HeightData | TerrainDataChangedMask::ColorData |
                                    TerrainDataChangedMask::SurfaceData,
                                "Height Data + Color Data + Surface Data"),
                        });
            }
        }
    }

    void CsvSpawnerEditorComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusConnect();

        if (m_showLabels)
        {
            AzFramework::ViewportDebugDisplayEventBus::Handler::BusConnect(AzToolsFramework::GetEntityContextId());
        }

        if (m_spawnOnComponentActivated && !m_flagSpawnEntitiesOnStartOnce)
        {
            AZ::TickBus::QueueFunction(
                [this]()
                {
                    // If there is no Terrain handlers (which means no active terrain in this level), just spawn entities on next available
                    // tick. Since terrain is initiated on tick, IsTerrainAvailable will return real information when used inside tick.
                    if (!IsTerrainAvailable() && !m_spawnOnTerrainUpdate)
                    {
                        m_flagSpawnEntitiesOnStartOnce = true;
                        SpawnEntities();
                    }
                });
        }
    }

    void CsvSpawnerEditorComponent::Deactivate()
    {
        m_spawnedTickets.clear();
        m_flagSpawnEntitiesOnStartOnce = false;

        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusDisconnect();
        AzFramework::ViewportDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void CsvSpawnerEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        // Create Game component
        const auto config = CsvSpawnerUtils::GetSpawnableAssetFromVector(m_spawnableAssetConfigurations);

        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        AZ::Data::AssetInfo sourceAssetInfo;
        bool ok{ false };
        AZStd::string watchFolder;
        AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;

        AssetSysReqBus::BroadcastResult(
            ok, &AssetSysReqBus::Events::GetSourceInfoBySourceUUID, m_csvAssetId.m_guid, sourceAssetInfo, watchFolder);
        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("CsvSpawnerEditorComponent", "Source of CSV file path: %s", sourcePath.c_str());

        auto spawnableEntityInfo = CsvSpawner::CsvSpawnerUtils::GetSpawnableEntityInfoFromCSV(sourcePath.String());

        gameEntity->CreateComponent<CsvSpawnerComponent>(config, spawnableEntityInfo, m_defaultSeed);

        // Destroy Editor's spawned entities
        m_spawnedTickets.clear();
    }

    void CsvSpawnerEditorComponent::OnTerrainDataChanged(const AZ::Aabb& dirtyRegion, TerrainDataChangedMask dataChangedMask)
    {
        // Ignore on update with selected flags
        if (static_cast<bool>(dataChangedMask & m_terrainDataChangedMask))
        {
            return;
        }

        if ((m_spawnOnComponentActivated && !m_flagSpawnEntitiesOnStartOnce) || m_spawnOnTerrainUpdate)
        {
            AZ::TickBus::QueueFunction(
                [this]()
                {
                    SpawnEntities();
                    m_flagSpawnEntitiesOnStartOnce = true;
                });
        }
    }

    void CsvSpawnerEditorComponent::SpawnEntities()
    {
        if (m_csvAssetId.IsValid() == false)
        {
            AZ_Error("CsvSpawnerEditorComponent", false, "CSV asset is not set");
            return;
        }

        m_spawnedTickets.clear();
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        AZ::Data::AssetInfo sourceAssetInfo;
        bool ok{ false };
        AZStd::string watchFolder;
        AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;

        AssetSysReqBus::BroadcastResult(
            ok, &AssetSysReqBus::Events::GetSourceInfoBySourceUUID, m_csvAssetId.m_guid, sourceAssetInfo, watchFolder);
        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("CsvSpawnerEditorComponent", "Source of CSV file path: %s", sourcePath.c_str());

        m_spawnableEntityInfo = CsvSpawner::CsvSpawnerUtils::GetSpawnableEntityInfoFromCSV(sourcePath.String());
        m_numberOfEntries = m_spawnableEntityInfo.size();

        AZ_Printf("CsvSpawnerEditorComponent", "Spawning spawnables, %d", m_numberOfEntries);

        const auto config = CsvSpawnerUtils::GetSpawnableAssetFromVector(m_spawnableAssetConfigurations);
        m_spawnedTickets =
            CsvSpawnerUtils::SpawnEntities(m_spawnableEntityInfo, config, m_defaultSeed, AzPhysics::EditorPhysicsSceneName, GetEntityId());
    }

    AZ::u32 CsvSpawnerEditorComponent::SetPropertyVisibilityByTerrain() const
    {
        return IsTerrainAvailable() ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 CsvSpawnerEditorComponent::RefreshUI()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZ::Crc32 CsvSpawnerEditorComponent::SpawnOnTerrainUpdateTriggered()
    {
        if (IsSpawnOnTerrainUpdateDisabled())
        {
            m_terrainDataChangedMask = TerrainDataChangedMask::All;
        }

        return RefreshUI();
    }

    AZ::Crc32 CsvSpawnerEditorComponent::OnTerrainFlagsChanged()
    {
        if (m_terrainDataChangedMask == TerrainDataChangedMask::All)
        {
            m_spawnOnTerrainUpdate = false;
        }

        return RefreshUI();
    }

    bool CsvSpawnerEditorComponent::IsSpawnOnTerrainUpdateDisabled() const
    {
        return !m_spawnOnTerrainUpdate;
    }

    bool CsvSpawnerEditorComponent::IsSpawnOnTerrainUpdateEnabled() const
    {
        return m_spawnOnTerrainUpdate;
    }

    void CsvSpawnerEditorComponent::OnShowLabelsChanged()
    {
        if (m_showLabels)
        {
            AzFramework::ViewportDebugDisplayEventBus::Handler::BusConnect(AzToolsFramework::GetEntityContextId());
        }
        else
        {
            AzFramework::ViewportDebugDisplayEventBus::Handler::BusDisconnect();
        }
    }

    void CsvSpawnerEditorComponent::OnSpawnButton()
    {
        SpawnEntities();
    }

    void CsvSpawnerEditorComponent::DisplayViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        const AZ::u32 stateBefore = debugDisplay.GetState();
        debugDisplay.CullOff();
        debugDisplay.DepthTestOff();
        debugDisplay.SetLineWidth(2.0f);

        debugDisplay.PushMatrix(transform);
        for (const auto& spawnableEntityInfo : m_spawnableEntityInfo)
        {
            const auto& location = spawnableEntityInfo.m_transform.GetTranslation();
            const auto& elevatedLocation = location + AZ::Vector3(0.0f, 0.0f, 3.0f);
            debugDisplay.SetColor(AZ::Colors::White);
            debugDisplay.DrawLine(location, elevatedLocation);
            AZStd::string text = AZStd::string::format("%d,%s", spawnableEntityInfo.m_id, spawnableEntityInfo.m_name.c_str());
            debugDisplay.DrawTextLabel(transform.TransformPoint(elevatedLocation), 1.0f, text.c_str());
        }
        debugDisplay.PopMatrix();
        debugDisplay.SetState(stateBefore);
    }
} // namespace CsvSpawner
