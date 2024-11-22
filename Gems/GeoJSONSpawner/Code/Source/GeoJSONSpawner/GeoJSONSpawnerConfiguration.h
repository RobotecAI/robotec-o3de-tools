/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "GeoJSONSpawner/GeoJSONSpawnerTypeIds.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Spawnable/Spawnable.h>

namespace GeoJSONSpawner
{

    struct GeoJSONSpawnerConfiguration
    {
        AZ_TYPE_INFO(GeoJSONSpawnerConfiguration, GeoJSONSpawnerConfigurationTypeId);

        static void Reflect(AZ::ReflectContext* context);

        AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> m_spawnableAssets;
        double m_altitude{ 0.0f };
    };

} // namespace GeoJSONSpawner
