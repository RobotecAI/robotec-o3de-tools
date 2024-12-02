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

#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace GeoJSONSpawner
{

    //! Interface for the GeoJSONSpawner
    //! Used to spawn, modify and despawn prefabs in runtime
    class GeoJSONSpawnerRequests : public AZ::ComponentBus
    {
    public:
        using BusIdType = AZ::EntityId;

        //! Spawn prefabs based on the information in provided GeoJSON
        //! @param rawJsonString - raw string with a GeoJSON to spawn
        virtual void Spawn(const AZStd::string& rawJsonString) = 0;

        //! Modify spawned prefabs based on the information in provided GeoJSON
        //! @param rawJsonString - raw string with a GeoJSON to modify
        //! If any of the given ID does not exist then modify action will be canceled
        virtual void Modify(const AZStd::string& rawJsonString) = 0;

        //! Delete all prefabs spawned with GeoJSONSpawner
        virtual void DeleteAll() = 0;

        //! Delete prefabs based on the information in provided GeoJSON
        //! @param rawJsonString - raw string with a GeoJSON to despawn
        virtual void DeleteById(const AZStd::string& rawJsonString) = 0;

        //! Get all used IDs together with a number of spawned prefabs associated with each ID
        //! @return AZStd::string with all IDs that are in use with a number of prefabs associated with each ID
        virtual AZStd::string GetIds() const = 0;
    };

    using GeoJSONSpawnerRequestBus = AZ::EBus<GeoJSONSpawnerRequests>;

} // namespace GeoJSONSpawner
