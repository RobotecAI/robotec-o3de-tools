/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerConfiguration.h"

#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Serialization/EditContext.h>

namespace GeoJSONSpawner
{
    void GeoJSONSpawnerConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerConfiguration>()
                ->Version(0)
                ->Field("SpawnableAssets", &GeoJSONSpawnerConfiguration::m_spawnableAssets)
                ->Field("Altitude", &GeoJSONSpawnerConfiguration::m_altitude);
            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GeoJSONSpawnerConfiguration>("GeoJSONSpawnerConfiguration", "GeoJSON Spawner Configuration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerConfiguration::m_spawnableAssets,
                        "SpawnableAssets",
                        "Spawnable Assets")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GeoJSONSpawnerConfiguration::m_altitude, "Altitude", "Altitude");
            }
        }
    }

} // namespace GeoJSONSpawner