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

#include <AzCore/EBus/EBus.h>

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

} // namespace GeoJSONSpawner
