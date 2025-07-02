/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with
 * the permission of the copyright holders.  If you encounter this file and do
 * not have permission, please contact the copyright holders and delete this
 * file.
 */

#pragma once

#include <CsvSpawner/CsvSpawnerTypeIds.h>

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace CsvSpawner::CsvSpawnerUtils
{

    //! Information about a locations to spawn an entity.
    //! This information is generated from CSV file and is used to spawn the entity.
    //! @param m_name is the name of the spawnable entity configuration and should
    //! be identical to name in @class SpawnableAssetConfiguration.
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
        uint64_t m_seed{ 0 }; //!< Optional seed value for randomization, in the context
                              //!< of one spawnable entity configuration, optional
    };

    //! Configuration for a spawnable asset
    //! This configuration is used to configure the spawnable asset before spawning
    //! it. The @param m_name is used to identify the spawnable asset configuration
    //! and should be identical to name given in CSV file. The class allows user to
    //! specify multiple spawnable assets for a given name and randomization
    //! parameters for position, rotation and scale. If more then one spawnable
    //! asset is specified for a given name, then one of the spawnable assets is
    //! randomly selected for spawning.
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
        bool m_placeOnTerrain{ false }; //!< Whether to raytrace to terrain and place
                                        //!< the entity on the terrain
        AzPhysics::CollisionGroups::Id m_selectedCollisionGroup; //!< To which collision group this target will
                                                                 //!< be attached

    private:
        [[nodiscard]] bool IsCollisionLayerEnabled() const;
        static AZ::Crc32 OnPlaceOnTerrainChanged();
    };

    //! This function create map of spawnable asset configuration from vector of
    //! spawnable asset configuration, where the key is the name of the spawnable
    //! asset configuration
    //! @param spawnableAssetConfigurations - vector of spawnable asset
    //! configuration
    //! @return map of spawnable asset configuration
    AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration> GetSpawnableAssetFromVector(
        AZStd::vector<CsvSpawnableAssetConfiguration> spawnableAssetConfigurations);

    //! Spawn entities from the vector of spawnable entity info
    //! @param entitiesToSpawn - vector of spawnable entity info
    //! @param spawnableAssetConfiguration - map of spawnable asset configuration
    //! @param defaultSeed - default seed value
    //! @param parentId - parent entity id to set for new entities
    //! @param physicsSceneName - physics scene name
    //! (AzPhysics::DefaultPhysicsSceneName or AzPhysics::EditorPhysicsSceneName)
    //! @return map of spawn created tickets
    AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> SpawnEntities(
        const AZStd::vector<CsvSpawnableEntityInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration>& spawnableAssetConfiguration,
        const AZ::u64 defaultSeed,
        const AZStd::string& physicsSceneName = AZStd::string(),
        AZ::EntityId parentId = AZ::EntityId());

    /**
     * @brief Flags representing the status of an CsvSpawner::Spawn() operation.
     *
     * SpawnStatus provides various status indicators for entity spawning.
     * These flags help track whether spawning was successful, stopped, or failed.
     */
    enum class SpawnStatus : uint8_t
    {
        Success = 0, ///< Operation succeeded.
        Fail = 1 << 0, ///< Generic failure.
        Stopped = 1 << 1, ///< Spawning was stopped prematurely but not necessarily a failure.
        Warning = 1 << 2, ///< An warning or error occurred during spawning (potentially recoverable).
    };

    /// Enable bitwise operations for SpawnStatus.
    AZ_DEFINE_ENUM_BITWISE_OPERATORS(SpawnStatus);

    /**
     * @brief Structure holding data related to CsvSpawner entity spawning.
     *
     * SpawnInfo contains information about the entities to be spawned, the physics scene
     * they belong to, and the parent entity responsible for the spawn operation.
     */
    struct SpawnInfo
    {
        AZ_TYPE_INFO(SpawnInfo, CsvSpawnerSpawnInfoTypeId);
        static void Reflect(AZ::ReflectContext* context);

        AZStd::vector<CsvSpawnableEntityInfo> m_entitiesToSpawn; ///< List of entities to spawn.
        AZStd::string m_physicsSceneName; ///< Name of the physics scene where entities will be spawned.
        AZ::EntityId m_spawnerParentEntityId; ///< Parent entity ID managing the spawn process.
    };

    [[nodiscard]] bool IsTerrainAvailable(); //!< @returns True if level has any valid Terrain handlers, false otherwise.
} // namespace CsvSpawner::CsvSpawnerUtils
