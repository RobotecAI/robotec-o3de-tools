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

#include <CsvSpawner/CsvSpawnerUtils.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace CsvSpawner
{
    /**
     * @brief Interface for handling entity spawn events for Csv Spawner.
     *
     * CsvSpawnerInterface is an Event Bus interface that notifies multiple
     * listeners when entity spawning begins and finishes.
     */
    class CsvSpawnerInterface : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(CsvSpawnerInterface, CsvSpawnerInterfaceTypeId);
        virtual ~CsvSpawnerInterface() = default;

        /**
         * @brief Called when entity spawning begins.
         * @param m_spawnInfo Struct holding information about entities to be spawned.
         */
        virtual void OnEntitiesSpawnBegin(CsvSpawnerUtils::SpawnInfo& m_spawnInfo)
        {
        }

        /**
         * @brief Called when entity spawning finishes.
         * @param m_spawnInfo Struct holding information about entities to be spawned.
         * @param m_statusCode Status code indicating success, failure and warnings of the spawn.
         */
        virtual void OnEntitiesSpawnFinished(CsvSpawnerUtils::SpawnInfo& m_spawnInfo, CsvSpawnerUtils::SpawnStatus& m_statusCode)
        {
        }

        /// EBus Configuration - Allows multiple listeners to handle events.
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
    };

    // Create an EBus using the notification interface
    using CsvSpawnerNotificationBus = AZ::EBus<CsvSpawnerInterface>;

    class CsvSpawnerNotificationBusHandler
        : public CsvSpawnerNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            CsvSpawnerNotificationBusHandler,
            CsvSpawnerNotificationBusHandlerTypeId,
            AZ::SystemAllocator,
            OnEntitiesSpawnBegin,
            OnEntitiesSpawnFinished);

        void OnEntitiesSpawnBegin(CsvSpawnerUtils::SpawnInfo& m_spawnInfo) override
        {
            Call(FN_OnEntitiesSpawnBegin, m_spawnInfo);
        }

        void OnEntitiesSpawnFinished(CsvSpawnerUtils::SpawnInfo& m_spawnInfo, CsvSpawnerUtils::SpawnStatus& m_statusCode) override
        {
            Call(FN_OnEntitiesSpawnFinished, m_spawnInfo, m_statusCode);
        }
    };
} // namespace CsvSpawner
