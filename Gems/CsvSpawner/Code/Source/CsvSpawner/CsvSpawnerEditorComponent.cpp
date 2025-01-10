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
#include "CsvSpawnerComponent.h"
#include "CsvSpawnerCsvParser.h"
#include "CsvSpawnerUtils.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Viewport/ViewportMessages.h>
#include <thread>

namespace CsvSpawner
{

    void CsvSpawnerEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<CsvSpawnerEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("CsvAssetId", &CsvSpawnerEditorComponent::m_csvAssetId)
                ->Field("NumberOfEntries", &CsvSpawnerEditorComponent::m_numberOfEntries)
                ->Field("SpawnableAssetConfigurations", &CsvSpawnerEditorComponent::m_spawnableAssetConfigurations)
                ->Field("DefaultSeed", &CsvSpawnerEditorComponent::m_defaultSeed)
                ->Field("ShowLabels", &CsvSpawnerEditorComponent::m_showLabels);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<CsvSpawnerEditorComponent>("CsvSpawnerEditorComponent", "CsvSpawnerEditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "CsvSpawnerEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "CsvSpawner")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnerEditorComponent::m_spawnableAssetConfigurations,
                        "Asset Config",
                        "Asset configuration")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_csvAssetId, "CSV Asset", "CSV asset")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Reload Csv", "Reload Csv")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Spawn")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorComponent::OnSpawnButton)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_numberOfEntries, "Number of entries", "")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_defaultSeed, "Default seed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CsvSpawnerEditorComponent::m_showLabels, "Show labels in Editor", "")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnerEditorComponent::OnOnShowLabelsChanged);
            }
        }
    }

    void CsvSpawnerEditorComponent::OnOnShowLabelsChanged()
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

    void CsvSpawnerEditorComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        if (m_showLabels)
        {
            AzFramework::ViewportDebugDisplayEventBus::Handler::BusConnect(AzToolsFramework::GetEntityContextId());
        }

        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusConnect();
    }

    void CsvSpawnerEditorComponent::Deactivate()
    {
        m_spawnedTickets.clear();

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

    void CsvSpawnerEditorComponent::OnTerrainDataCreateEnd()
    {
        if (!m_terrainReady) // Init only once, even if level have multiple terrains
        {
            AZ::TickBus::QueueFunction([this]()
            {
                SpawnEntities();
            });

            m_terrainReady = true;
        }
    }

    void CsvSpawnerEditorComponent::OnTerrainDataDestroyBegin()
    {
        m_terrainReady = false;
        AzFramework::Terrain::TerrainDataNotificationBus::Handler::BusDisconnect();
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
