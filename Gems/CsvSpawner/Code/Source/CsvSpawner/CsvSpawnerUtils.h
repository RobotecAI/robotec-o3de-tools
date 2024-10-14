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

#include "CsvSpawner/CsvSpawnerTypeIds.h"

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace CsvSpawner::CsvSpawnerUtils
{

    //! Information about a locations to spawn an entity.
    //! This information is generated from CSV file and is used to spawn the entity.
    //! @param m_name is the name of the spawnable entity configuration and should be identical to name in @class
    //! SpawnableAssetConfiguration.
    class CsvSpawnableEntityInfo
    {
    public:
        AZ_RTTI(CsvSpawnableEntityInfo, CsvSpawnableEntityInfoTypeId);
        static void Reflect(AZ::ReflectContext* context);
        CsvSpawnableEntityInfo() = default;
        virtual ~CsvSpawnableEntityInfo() = default;

        unsigned int m_id; //!< Optional ID for the entity
        AZ::Transform m_transform; //!< Transform of the entity, mandatory
        AZStd::string m_name; //!< Name of the spawnable entity configuration, mandatory
        uint64_t m_seed{ 0 }; //!< Optional seed value for randomization, in the context of one spawnable entity configuration, optional
    };

    //! Configuration for a spawnable asset
    //! This configuration is used to configure the spawnable asset before spawning it.
    //! The @param m_name is used to identify the spawnable asset configuration and should be identical to name given in CSV file.
    //! The class allows user to specify multiple spawnable assets for a given name and randomization parameters for position, rotation and
    //! scale. If more then one spawnable asset is specified for a given name, then one of the spawnable assets is randomly selected for
    //! spawning.
    class CsvSpawnableAssetConfiguration
    {
    public:
        AZ_RTTI(CsvSpawnableAssetConfiguration, CsvSpawnableAssetConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);
        CsvSpawnableAssetConfiguration() = default;
        virtual ~CsvSpawnableAssetConfiguration() = default;

        AZStd::string m_name; //!< Name of the spawnable field
        AZStd::vector<AZ::Data::Asset<AzFramework::Spawnable>> m_spawnables; //!< List of spawnable assets
        AZ::Vector3 m_positionStdDev{ 0.f }; //!< Standard deviation for position
        AZ::Vector3 m_rotationStdDev{ 0.f }; //!< Standard deviation for rotation
        float m_scaleStdDev{ 0.1f }; //!< Standard deviation for scale
        bool m_placeOnTerrain{ false }; //!< Whether to raytrace to terrain and place the entity on the terrain
		AzPhysics::CollisionLayer m_selectedCollisionLayer; //!< To which collision layer this target will be attached

    private:
        bool IsCollisionLayerEnabled() const;
        AZ::Crc32 OnPlaceOnTerrainChanged();
    };

    //! This function create map of spawnable asset configuration from vector of spawnable asset configuration, where
    //! the key is the name of the spawnable asset configuration
    //! @param spawnableAssetConfigurations - vector of spawnable asset configuration
    //! @return map of spawnable asset configuration
    AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration> GetSpawnableAssetFromVector(
        AZStd::vector<CsvSpawnableAssetConfiguration> spawnableAssetConfigurations);

    //! Spawn entities from the vector of spawnable entity info
    //! @param entitiesToSpawn - vector of spawnable entity info
    //! @param spawnableAssetConfiguration - map of spawnable asset configuration
    //! @param defaultSeed - default seed value
    //! @param parentId - parent entity id to set for new entities
    //! @param physicsSceneName - physics scene name (AzPhysics::DefaultPhysicsSceneName or AzPhysics::EditorPhysicsSceneName)
    //! @return map of spawn created tickets
    AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> SpawnEntities(
        const AZStd::vector<CsvSpawnableEntityInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration>& spawnableAssetConfiguration,
        const AZ::u64 defaultSeed,
        const AZStd::string& physicsSceneName = AZStd::string(),
        AZ::EntityId parentId = AZ::EntityId());

}; // namespace CsvSpawner::CsvSpawnerUtils
