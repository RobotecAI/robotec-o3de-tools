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

#include "GeoJSONSpawnerUtils.h"

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/Component.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

#include <GeoJSONSpawner/GeoJSONSpawnerBus.h>
#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

namespace GeoJSONSpawner
{

    class GeoJSONSpawnerComponent
        : public AZ::Component
        , public GeoJSONSpawnerRequestBus::Handler
    {
    public:
        AZ_COMPONENT(GeoJSONSpawnerComponent, GeoJSONSpawnerComponentTypeId);

        GeoJSONSpawnerComponent() = default;
        explicit GeoJSONSpawnerComponent(
            const AZStd::unordered_map<AZStd::string, GeoJSONUtils::GeoJSONSpawnableAssetConfiguration>& spawnableAssetConfigurations,
            const AZStd::string& geoJsonFilePath,
            AZ::u64 defaultSeed);
        ~GeoJSONSpawnerComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        // GeoJSONSpawnerRequestBus::Handler overrides...
        void Spawn(const AZStd::string& rawJsonString) override;

    private:
        void SpawnEntities();

        AZStd::unordered_map<AZStd::string, GeoJSONUtils::GeoJSONSpawnableAssetConfiguration> m_spawnableAssetConfigurations;
        AZ::u64 m_defaultSeed;
        AZStd::string m_geoJsonFilePath;

        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> m_spawnableTickets;
        AZStd::vector<GeoJSONUtils::GeoJSONSpawnableEntityInfo> m_spawnableEntityInfo;
    };
} // namespace GeoJSONSpawner