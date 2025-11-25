/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "RobotecGeoJSONSpawnerEditorComponent.h"
#include "EditorConfigurations/RobotecGeoJSONSpawnerEditorTerrainSettingsConfig.h"
#include "RobotecGeoJSONSpawnerComponent.h"

#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Viewport/ViewportMessages.h>

namespace RobotecGeoJSONSpawner
{
    void RobotecGeoJSONSpawnerEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            RobotecGeoJSONSpawnerEditorTerrainSettingsConfig::Reflect(context);

            serializeContext->Class<RobotecGeoJSONSpawnerEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(0)
                ->Field("GeoJSONAssetId", &RobotecGeoJSONSpawnerEditorComponent::m_geoJsonAssetId)
                ->Field("Configuration", &RobotecGeoJSONSpawnerEditorComponent::m_spawnableAssetConfigurations)
                ->Field("DefaultSeed", &RobotecGeoJSONSpawnerEditorComponent::m_defaultSeed)
                ->Field("ShowLabels", &RobotecGeoJSONSpawnerEditorComponent::m_showLabels)
                ->Field("ConfigTerrainSettings", &RobotecGeoJSONSpawnerEditorComponent::m_terrainSettingsConfig);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<RobotecGeoJSONSpawnerEditorComponent>("RobotecGeoJSONSpawnerEditorComponent", "Gem Spawner Editor Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSON Spawner Editor Component")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RobotecGeoJSONSpawnerEditorComponent::m_geoJsonAssetId,
                        "GeoJSON Asset Id",
                        "ID of the asset containing GeoJSON that will be spawned.")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Reload GeoJSON", "Reload GeoJSON")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Spawn")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &RobotecGeoJSONSpawnerEditorComponent::OnSpawnButton)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RobotecGeoJSONSpawnerEditorComponent::m_spawnableAssetConfigurations,
                        "Spawnable Asset Configurations",
                        "Spawnable Asset Configurations.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RobotecGeoJSONSpawnerEditorComponent::m_defaultSeed,
                        "Default seed",
                        "Default seed used for randomization.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RobotecGeoJSONSpawnerEditorComponent::m_showLabels,
                        "Show labels in Editor",
                        "Show labels in Editor.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &RobotecGeoJSONSpawnerEditorComponent::OnShowLabelsChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RobotecGeoJSONSpawnerEditorComponent::m_terrainSettingsConfig,
                        "Spawn Behaviour Settings",
                        "Settings to configure spawn behaviour in editor.");
            }
        }
    }

    void RobotecGeoJSONSpawnerEditorComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusConnect();
        if (m_showLabels)
        {
            AzFramework::ViewportDebugDisplayEventBus::Handler::BusConnect(AzToolsFramework::GetEntityContextId());
        }

        if (m_terrainSettingsConfig.m_spawnOnComponentActivated && !m_terrainSettingsConfig.m_flagSpawnEntitiesOnStartOnce)
        {
            AZ::TickBus::QueueFunction(
                [this]()
                {
                    // If there is no Terrain handlers (which means no active terrain in this level), just spawn entities on next available
                    // tick. Since terrain is initiated on tick, IsTerrainAvailable will return real information when used inside tick.
                    if (!GeoJSONUtils::IsTerrainAvailable() && !m_terrainSettingsConfig.m_spawnOnTerrainUpdate)
                    {
                        m_terrainSettingsConfig.m_flagSpawnEntitiesOnStartOnce = true;
                        SpawnEntities();
                    }
                });
        }
    }

    void RobotecGeoJSONSpawnerEditorComponent::Deactivate()
    {
        m_spawnedTicketsGroups.clear();
        m_terrainSettingsConfig.m_flagSpawnEntitiesOnStartOnce = false;

        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusDisconnect();
        AzFramework::ViewportDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void RobotecGeoJSONSpawnerEditorComponent::SpawnEntities()
    {
        if (!m_geoJsonAssetId.IsValid())
        {
            AZ_Error("RobotecGeoJSONSpawnerEditorComponent", false, "JSON asset is not set.");
            return;
        }

        m_spawnedTicketsGroups.clear();
        AZ::Data::AssetInfo sourceAssetInfo;
        bool ok{ false };
        AZStd::string watchFolder;

        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
            ok,
            &AzToolsFramework::AssetSystemRequestBus::Events::GetSourceInfoBySourceUUID,
            m_geoJsonAssetId.m_guid,
            sourceAssetInfo,
            watchFolder);

        if (!ok)
        {
            AZ_Error("RobotecGeoJSONSpawnerEditorComponent", false, "Cannot find asset source.");
            return;
        }

        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("RobotecGeoJSONSpawnerEditorComponent", "Source of GeoJSON file path: %s", sourcePath.c_str());

        auto spawnableAssetConfigurationsMap = GeoJSONUtils::GetSpawnableAssetFromVector(m_spawnableAssetConfigurations);
        const auto featureObjectInfo = GeoJSONUtils::ParseJSONFromFile(sourcePath.c_str());
        m_spawnableEntityInfo =
            GeoJSONUtils::GetSpawnableEntitiesFromFeatureObjectVector(featureObjectInfo, spawnableAssetConfigurationsMap);

        auto ticketsToSpawn = GeoJSONUtils::PrepareTicketsToSpawn(
            m_spawnableEntityInfo,
            spawnableAssetConfigurationsMap,
            m_defaultSeed,
            []() // empty lambda as Editor Component does not need to care about ticket spawn/despawn confirmation
            {
            },
            AzPhysics::EditorPhysicsSceneName,
            GetEntityId());
        m_spawnedTicketsGroups = GeoJSONUtils::SpawnEntities(ticketsToSpawn);
    }

    void RobotecGeoJSONSpawnerEditorComponent::OnSpawnButton()
    {
        SpawnEntities();
    }

    void RobotecGeoJSONSpawnerEditorComponent::OnShowLabelsChanged()
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

    void RobotecGeoJSONSpawnerEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        AZ::Data::AssetInfo sourceAssetInfo;
        bool isSourceFound{ false };
        AZStd::string watchFolder;

        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
            isSourceFound,
            &AzToolsFramework::AssetSystemRequestBus::Events::GetSourceInfoBySourceUUID,
            m_geoJsonAssetId.m_guid,
            sourceAssetInfo,
            watchFolder);

        if (!isSourceFound)
        {
            AZ_Error("RobotecGeoJSONSpawnerEditorComponent", false, "Cannot find asset source.");
            return;
        }

        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("RobotecGeoJSONSpawnerEditorComponent", "Source of GeoJSON file path: %s", sourcePath.c_str());

        auto spawnableAssetConfigurationsMap = GeoJSONUtils::GetSpawnableAssetFromVector(m_spawnableAssetConfigurations);
        const auto featureObjectInfo = GeoJSONUtils::ParseJSONFromFile(sourcePath.c_str());
        auto spawnableEntitiesInfo =
            GeoJSONUtils::GetSpawnableEntitiesFromFeatureObjectVector(featureObjectInfo, spawnableAssetConfigurationsMap);
        gameEntity->CreateComponent<RobotecGeoJSONSpawnerComponent>(
            spawnableAssetConfigurationsMap, sourceAssetInfo.m_relativePath.c_str(), m_defaultSeed);
        m_spawnedTicketsGroups.clear();
    }

    void RobotecGeoJSONSpawnerEditorComponent::OnTerrainDataChanged(
        [[maybe_unused]] const AZ::Aabb& dirtyRegion, TerrainDataChangedMask dataChangedMask)
    {
        // Ignore on update with selected flags
        if (static_cast<bool>(dataChangedMask & m_terrainSettingsConfig.m_terrainMasksToIgnore))
        {
            return;
        }

        if ((m_terrainSettingsConfig.m_spawnOnComponentActivated && !m_terrainSettingsConfig.m_flagSpawnEntitiesOnStartOnce) ||
            m_terrainSettingsConfig.m_spawnOnTerrainUpdate)
        {
            AZ::TickBus::QueueFunction(
                [this]()
                {
                    SpawnEntities();
                    m_terrainSettingsConfig.m_flagSpawnEntitiesOnStartOnce = true;
                });
        }
    }

    void RobotecGeoJSONSpawnerEditorComponent::DisplayViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        const AZ::u32 stateBefore = debugDisplay.GetState();
        debugDisplay.CullOff();
        debugDisplay.DepthTestOff();
        debugDisplay.SetLineWidth(2.0f);

        debugDisplay.PushMatrix(transform);
        for (const auto& entityInfo : m_spawnableEntityInfo)
        {
            const AZStd::string name = entityInfo.m_name;
            const int id = entityInfo.m_id;
            for (const auto& point : entityInfo.m_positions)
            {
                const AZ::Vector3 labelPosition = point.GetTranslation() + AZ::Vector3(0.0f, 0.0f, 3.0f);
                debugDisplay.SetColor(AZ::Colors::White);
                debugDisplay.DrawLine(point.GetTranslation(), labelPosition);
                AZStd::string labelText = AZStd::string::format("%d,%s", id, name.c_str());
                debugDisplay.DrawTextLabel(labelPosition, 1.0f, labelText.c_str());
            }
        }
        debugDisplay.PopMatrix();
        debugDisplay.SetState(stateBefore);
    }

} // namespace RobotecGeoJSONSpawner
