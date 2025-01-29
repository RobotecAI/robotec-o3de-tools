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

#include <AzCore/EBus/EBus.h>
#include <AzCore/std/string/string.h>
#include <CsvSpawner/CsvSpawnerUtils.h>

namespace CsvSpawner
{

    /**
     * @brief Flags representing the status of an CsvSpawner::Spawn() operation.
     *
     * SpawnStatusCode provides various status indicators for entity spawning.
     * These flags help track whether spawning was successful, stopped, or failed.
     */
    enum class SpawnStatusCode : uint8_t
    {
        Success = 0, ///< Operation succeeded.
        Fail = 1 << 0, ///< Generic failure.
        SpawnStopped = 1 << 1, ///< Spawning was stopped prematurely but not necessarily a failure.
        ErrorOccurred = 1 << 2, ///< An error occurred during spawning (potentially recoverable).
    };

    /// Enable bitwise operations for SpawnStatusCode.
    AZ_DEFINE_ENUM_BITWISE_OPERATORS(SpawnStatusCode);

    /**
     * @brief Structure holding data related to CsvSpawner entity spawning.
     *
     * SpawnInfo contains information about the entities to be spawned, the physics scene
     * they belong to, and the parent entity responsible for the spawn operation.
     */
    struct SpawnInfo
    {
        const AZStd::vector<CsvSpawnerUtils::CsvSpawnableEntityInfo>& m_entitiesToSpawn; ///< List of entities to spawn.
        const AZStd::string& m_physicsSceneName; ///< Name of the physics scene where entities will be spawned.
        const AZ::EntityId& m_spawnerParentEntityId; ///< Parent entity ID managing the spawn process.
    };

    /**
     * @brief Interface for handling entity spawn events for Csv Spawner.
     *
     * CsvSpawnerInterface is an Event Bus interface that notifies multiple
     * listeners when entity spawning begins and finishes.
     */
    class CsvSpawnerInterface : public AZ::EBusTraits
    {
    public:
        virtual ~CsvSpawnerInterface() = default;

        /**
         * @brief Called when entity spawning begins.
         * @param m_spawnInfo Struct holding information about entities to be spawned.
         */
        virtual void OnEntitiesSpawnBegin(const SpawnInfo& m_spawnInfo)
        {
        }

        /**
         * @brief Called when entity spawning finishes.
         * @param m_spawnInfo Struct holding information about entities to be spawned.
         * @param m_statusCode Status code indicating success, failure and warnings of the spawn.
         */
        virtual void OnEntitiesSpawnFinished(const SpawnInfo& m_spawnInfo, const SpawnStatusCode& m_statusCode)
        {
        }

        /// EBus Configuration - Allows multiple listeners to handle events.
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
    };

    // Create an EBus using the notification interface
    using CsvSpawnerNotificationBus = AZ::EBus<CsvSpawnerInterface>;

} // namespace CsvSpawner
