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

#include "GeoJSONSpawnerComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>

namespace GeoJSONSpawner
{
    void GeoJSONSpawnerEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerEditorComponent, AzToolsFramework::Components::EditorComponentBase>()->Version(0)->Field(
                "Configuration", &GeoJSONSpawnerEditorComponent::m_configuration);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GeoJSONSpawnerEditorComponent>("GeoJSONSpawnerEditorComponent", "Gem Spawner Editor Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSON Spawner Editor Component")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerEditorComponent::m_configuration,
                        "GeoJSONSpawner Configuration",
                        "GeoJSONSpawner Configuration")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Reload GeoJSON", "Reload GeoJSON")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Spawn")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GeoJSONSpawnerEditorComponent::OnSpawnButton);
            }
        }
    }

    void GeoJSONSpawnerEditorComponent::SpawnEntities()
    {
        if (!m_configuration.m_geoJsonAssetId.IsValid())
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
            m_configuration.m_geoJsonAssetId.m_guid,
            sourceAssetInfo,
            watchFolder);

        if (!ok)
        {
            AZ_Error("GeoJSONSpawnerEditorComponent", false, "Cannot find asset source.");
            return;
        }

        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("GeoJSONSpawnerEditorComponent", "Source of GeoJSON file path: %s", sourcePath.c_str());

        auto result = GeoJSONUtils::ParseJSONFromFile(sourcePath.c_str());
        m_spawnedTicketsGroups = GeoJSONUtils::SpawnEntities(result, m_configuration.m_spawnableAssets, GetEntityId());
    }

    void GeoJSONSpawnerEditorComponent::OnSpawnButton()
    {
        SpawnEntities();
    }

    void GeoJSONSpawnerEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<GeoJSONSpawnerComponent>(m_configuration);
    }

} // namespace GeoJSONSpawner