/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerEditorComponent.h"

#include "AzCore/Component/TransformBus.h"
#include "GeoJSONSpawnerComponent.h"

#include <AzCore/Component/TickBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Viewport/ViewportMessages.h>

namespace GeoJSONSpawner
{
    void GeoJSONSpawnerEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(0)
                ->Field("GeoJSONAssetId", &GeoJSONSpawnerEditorComponent::m_geoJsonAssetId)
                ->Field("Configuration", &GeoJSONSpawnerEditorComponent::m_spawnableAssetConfigurations)
                ->Field("DefaultSeed", &GeoJSONSpawnerEditorComponent::m_defaultSeed)
                ->Field("ShowLabels", &GeoJSONSpawnerEditorComponent::m_showLabels);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GeoJSONSpawnerEditorComponent>("GeoJSONSpawnerEditorComponent", "Gem Spawner Editor Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSON Spawner Editor Component")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerEditorComponent::m_geoJsonAssetId,
                        "GeoJSON Asset Id",
                        "GeoJSON Asset Id")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Reload GeoJSON", "Reload GeoJSON")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Spawn")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GeoJSONSpawnerEditorComponent::OnSpawnButton)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerEditorComponent::m_spawnableAssetConfigurations,
                        "Spawnable Asset Configurations",
                        "Spawnable Asset Configurations")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &GeoJSONSpawnerEditorComponent::m_defaultSeed, "Default seed", "Default seed")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerEditorComponent::m_showLabels,
                        "Show labels in Editor",
                        "Show labels in Editor")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GeoJSONSpawnerEditorComponent::OnShowLabelsChanged);
            }
        }
    }

    void GeoJSONSpawnerEditorComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        if (m_showLabels)
        {
            AzFramework::ViewportDebugDisplayEventBus::Handler::BusConnect(AzToolsFramework::GetEntityContextId());
        }

        AZ::TickBus::QueueFunction(
            [this]()
            {
                SpawnEntities();
            });
    }

    void GeoJSONSpawnerEditorComponent::Deactivate()
    {
        m_spawnedTicketsGroups.clear();

        AzFramework::ViewportDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void GeoJSONSpawnerEditorComponent::SpawnEntities()
    {
        if (!m_geoJsonAssetId.IsValid())
        {
            AZ_Error("GeoJSONSpawnerEditorComponent", false, "JSON asset is not set.");
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
            AZ_Error("GeoJSONSpawnerEditorComponent", false, "Cannot find asset source.");
            return;
        }

        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("GeoJSONSpawnerEditorComponent", "Source of GeoJSON file path: %s", sourcePath.c_str());

        auto spawnableAssetConfigurationsMap = GeoJSONUtils::GetSpawnableAssetFromVector(m_spawnableAssetConfigurations);
        const auto geometryObjectInfo = GeoJSONUtils::ParseJSONFromFile(sourcePath.c_str());
        m_spawnableEntityInfo =
            GeoJSONUtils::GetSpawnableEntitiesFromGeometryObjectVector(geometryObjectInfo, spawnableAssetConfigurationsMap);

        m_spawnedTicketsGroups = GeoJSONUtils::SpawnEntities(
            m_spawnableEntityInfo, spawnableAssetConfigurationsMap, m_defaultSeed, AzPhysics::EditorPhysicsSceneName, GetEntityId());
    }

    void GeoJSONSpawnerEditorComponent::OnSpawnButton()
    {
        SpawnEntities();
    }

    void GeoJSONSpawnerEditorComponent::OnShowLabelsChanged()
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

    void GeoJSONSpawnerEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
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
            AZ_Error("GeoJSONSpawnerEditorComponent", false, "Cannot find asset source.");
            return;
        }

        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("GeoJSONSpawnerEditorComponent", "Source of GeoJSON file path: %s", sourcePath.c_str());

        auto spawnableAssetConfigurationsMap = GeoJSONUtils::GetSpawnableAssetFromVector(m_spawnableAssetConfigurations);
        const auto geometryObjectInfo = GeoJSONUtils::ParseJSONFromFile(sourcePath.c_str());
        auto spawnableEntitiesInfo =
            GeoJSONUtils::GetSpawnableEntitiesFromGeometryObjectVector(geometryObjectInfo, spawnableAssetConfigurationsMap);
        gameEntity->CreateComponent<GeoJSONSpawnerComponent>(spawnableAssetConfigurationsMap, sourcePath.c_str(), m_defaultSeed);
        m_spawnedTicketsGroups.clear();
    }

    void GeoJSONSpawnerEditorComponent::DisplayViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        const AZ::u32 stateBefore = debugDisplay.GetState();
        debugDisplay.CullOff();
        debugDisplay.DepthTestOff();
        debugDisplay.SetLineWidth(2.0f);

        debugDisplay.PushMatrix(transform);
        for (const auto& geometryObject : m_spawnableEntityInfo)
        {
            const AZStd::string name = geometryObject.m_name;
            const int id = geometryObject.m_id;
            for (const auto& point : geometryObject.m_positions)
            {
                const AZ::Vector3 labelPosition = point + AZ::Vector3(0.0f, 0.0f, 3.0f);
                debugDisplay.SetColor(AZ::Colors::White);
                debugDisplay.DrawLine(point, labelPosition);
                AZStd::string labelText = AZStd::string::format("%d,%s", id, name.c_str());
                debugDisplay.DrawTextLabel(labelPosition, 1.0f, labelText.c_str());
            }
        }
        debugDisplay.PopMatrix();
        debugDisplay.SetState(stateBefore);
    }

} // namespace GeoJSONSpawner