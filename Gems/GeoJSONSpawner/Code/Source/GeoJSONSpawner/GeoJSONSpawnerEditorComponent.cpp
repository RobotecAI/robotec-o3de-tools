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
                        "GeoJSONSpawner Configuration");
            }
        }
    }

    void GeoJSONSpawnerEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<GeoJSONSpawnerComponent>(m_configuration);
    }

} // namespace GeoJSONSpawner