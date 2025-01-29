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

    // Enum with flags for spawning status
    enum class SpawnStatusCode : uint8_t
    {
        Success = 0, // Operation succeeded.
        Fail = 1 << 0, // Generic failure.
        SpawnStopped = 1 << 1, // Spawning was stopped prematurely but not necessarily a failure.
        ErrorOccurred = 1 << 2, // An error occurred during spawning (potentially recoverable).
    };

    AZ_DEFINE_ENUM_BITWISE_OPERATORS(SpawnStatusCode);

    // Structure to hold spawn data
    struct SpawnInfo
    {
        const AZStd::vector<CsvSpawnerUtils::CsvSpawnableEntityInfo>& m_entitiesToSpawn;
        const AZStd::string& m_physicsSceneName;
        const AZ::EntityId& m_spawnerParentEntityId;
    };

    class CsvSpawnerInterface : public AZ::EBusTraits
    {
    public:
        virtual ~CsvSpawnerInterface() = default;

        virtual void OnEntitiesSpawnBegin(const SpawnInfo& m_spawnInfo) = 0;

        virtual void OnEntitiesSpawnFinished(const SpawnInfo& m_spawnInfo, const SpawnStatusCode& m_statusCode) = 0;

        // EBus Configuration - allow multiple listeners
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
    };

    // Create an EBus using the notification interface
    using CsvSpawnerNotificationBus = AZ::EBus<CsvSpawnerInterface>;

} // namespace CsvSpawner
