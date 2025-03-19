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
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/std/string/string.h>

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
     * @brief Interface for handling entity spawn events in GeoJSON Spawner.
     *
     * GeoJSONSpawnerInterface is an Event Bus (EBus) interface that notifies multiple listeners
     * when entity spawning and despawning begins or finishes.
     */
    class GeoJSONSpawnerInterface : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(GeoJSONSpawnerInterface, GeoJSONSpawnerInterfaceTypeId);
        virtual ~GeoJSONSpawnerInterface() = default;

        /**
         * @brief Called when entity spawning begins.
         *
         * Notifies multiple listeners that a batch entity spawning process has started.
         */
        virtual void OnEntitiesSpawnBegin() = 0;

        /**
         * @brief Called when entity spawning finishes.
         *
         * @param spawnedEntityTickets A map linking an integer ID to a list of spawned entity tickets.
         * @param m_statusCode Status code indicating success, failure, or warnings.
         *
         * This function is triggered when all entities in a spawn operation are processed.
         */
        virtual void OnEntitiesSpawnFinished(
            AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& spawnedEntityTickets,
            GeoJSONUtils::SpawnStatus m_statusCode) = 0;

        /**
         * @brief Called when entity despawning begins.
         *
         * Notifies multiple listeners that a batch entity despawning process has started.
         */
        virtual void OnEntitiesDespawnBegin() = 0;

        /**
         * @brief Called when entity despawning finishes.
         *
         * @param despawnedEntityTickets A map linking an integer ID to a list of despawned entity tickets.
         * @param m_statusCode Status code indicating success, failure, or warnings.
         *
         * This function is triggered when all entities in a despawn operation are processed.
         */
        virtual void OnEntitiesDespawnFinished(
            AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& despawnedEntityTickets,
            GeoJSONUtils::SpawnStatus m_statusCode) = 0;

        /**
         * @brief Called when an individual entity is successfully spawned.
         *
         * @param spawnedEntityTicket The spawn ticket representing the successfully spawned entity.
         *
         * This function is triggered per entity when it spawns.
         */
        virtual void OnEntitySpawn(AzFramework::EntitySpawnTicket& spawnedEntityTicket) = 0;

        /**
         * @brief Called when an individual entity is successfully despawned.
         *
         * @param despawnedEntityTicket The spawn ticket representing the successfully despawned entity.
         *
         * This function is triggered per entity when it despawns.
         */
        virtual void OnEntityDespawn(AzFramework::EntitySpawnTicket& despawnedEntityTicket) = 0;

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
            OnEntitiesSpawnFinished,
            OnEntitiesDespawnBegin,
            OnEntitiesDespawnFinished,
            OnEntitySpawn,
            OnEntityDespawn);

        void OnEntitiesSpawnBegin() override
        {
            Call(FN_OnEntitiesSpawnBegin);
        }

        void OnEntitiesSpawnFinished(
            AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& spawnedEntityTickets,
            GeoJSONUtils::SpawnStatus m_statusCode) override
        {
            Call(FN_OnEntitiesSpawnFinished, spawnedEntityTickets, m_statusCode);
        }

        void OnEntitiesDespawnBegin() override
        {
            Call(FN_OnEntitiesDespawnBegin);
        }

        void OnEntitiesDespawnFinished(
            AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& despawnedEntityTickets,
            GeoJSONUtils::SpawnStatus m_statusCode) override
        {
            Call(FN_OnEntitiesDespawnFinished, despawnedEntityTickets, m_statusCode);
        }

        void OnEntitySpawn(AzFramework::EntitySpawnTicket& spawnedEntityTicket) override
        {
            Call(FN_OnEntitySpawn, spawnedEntityTicket);
        }

        void OnEntityDespawn(AzFramework::EntitySpawnTicket& despawnedEntityTicket) override
        {
            Call(FN_OnEntityDespawn, despawnedEntityTicket);
        }
    };
} // namespace GeoJSONSpawner
