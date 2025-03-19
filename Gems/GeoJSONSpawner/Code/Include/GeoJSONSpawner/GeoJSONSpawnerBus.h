/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "GeoJSONSpawnerTypeIds.h"
#include <GeoJSONSpawner/GeoJSONSpawnerUtils.h>

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/IO/Path/Path_fwd.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace GeoJSONSpawner
{
    using Result = AZ::Outcome<void, AZStd::string>;
    using GetIdsResult = AZ::Outcome<AZStd::string, AZStd::string>;

    //! Interface for the GeoJSONSpawner
    //! Used for spawning, despawning and modifying prefabs described in the GeoJSON file
    class GeoJSONSpawnerRequests : public AZ::ComponentBus
    {
    public:
        using BusIdType = AZ::EntityId;
        using MutexType = AZStd::mutex;

        //! Spawn prefabs based on the information in provided GeoJSON. If there are already spawned entities on the scene, Component will
        //! despawn all of them before spawning new ones.
        //! @param rawJsonString - raw string with a GeoJSON to spawn
        virtual Result SpawnWithRawString(const AZStd::string& rawJsonString) = 0;

        //! Spawn prefabs using file from a given path. If there are already spawned entities on the scene, Component will despawn all of
        //! them before spawning new ones.
        //! @param assetPath - path to a file with a GeoJSON to spawn
        virtual Result SpawnWithAssetPath(const AZ::IO::Path& assetPath) = 0;

        //! Modify spawned prefabs based on the information in provided GeoJSON. Component will despawn all entities that are associated
        //! with given group ids before spawning modified ones.
        //! @param rawJsonString - raw string with a GeoJSON to modify
        //! If any of the given ID does not exist then modify action will be canceled
        virtual Result Modify(const AZStd::string& rawJsonString) = 0;

        //! Delete all prefabs spawned with GeoJSONSpawner
        virtual Result DeleteAll() = 0;

        //! Delete prefabs based on the information in provided GeoJSON
        //! @param idsToDelete - unordered set containing IDs to delete
        virtual Result DeleteById(const AZStd::unordered_set<int>& idsToDelete) = 0;

        //! Get all used IDs together with a number of spawned prefabs associated with each ID
        //! @return AZStd::string with all IDs that are in use with a number of prefabs associated with each ID
        virtual GetIdsResult GetIds() const = 0;
    };

    using GeoJSONSpawnerRequestBus = AZ::EBus<GeoJSONSpawnerRequests>;

    /**
    * @brief Interface for handling entity spawn events for GeoJSON Spawner.
    *
    * GeoJSONSpawnerInterface is an Event Bus interface that notifies multiple
    * listeners when entity spawning begins and finishes.
    */
    class GeoJSONSpawnerInterface : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(GeoJSONSpawnerInterface, GeoJSONSpawnerNotificationsTypeId);
        virtual ~GeoJSONSpawnerInterface() = default;

        /**
         * @brief Called when entity spawning begins.
         * @param m_spawnInfo Struct holding information about entities to be spawned.
         */
        virtual void OnEntitiesSpawnBegin(GeoJSONUtils::SpawnInfo& m_spawnInfo) = 0;

        /**
         * @brief Called when entity spawning finishes.
         * @param m_spawnInfo Struct holding information about entities to be spawned.
         * @param m_statusCode Status code indicating success, failure and warnings of the spawn.
         */
        virtual void OnEntitiesSpawnFinished(GeoJSONUtils::SpawnInfo& m_spawnInfo, GeoJSONUtils::SpawnStatus m_statusCode) = 0;

        /// EBus Configuration - Allows multiple listeners to handle events.
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
    };
    // Create an EBus using the notification interface
    using GeoJSONSpawnerNotificationBus = AZ::EBus<GeoJSONSpawnerInterface>;

    class GeoJSONSpawnerNotificationBusHandler
        : public GeoJSONSpawnerNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            GeoJSONSpawnerNotificationBusHandler,
            GeoJSONSpawnerNotificationBusHandlerTypeId,
            AZ::SystemAllocator,
            OnEntitiesSpawnBegin,
            OnEntitiesSpawnFinished);

        void OnEntitiesSpawnBegin(GeoJSONUtils::SpawnInfo& m_spawnInfo) override
        {
            Call(FN_OnEntitiesSpawnBegin, m_spawnInfo);
        }

        void OnEntitiesSpawnFinished(GeoJSONUtils::SpawnInfo& m_spawnInfo, GeoJSONUtils::SpawnStatus m_statusCode) override
        {
            Call(FN_OnEntitiesSpawnFinished, m_spawnInfo, m_statusCode);
        }
    };
} // namespace GeoJSONSpawner
