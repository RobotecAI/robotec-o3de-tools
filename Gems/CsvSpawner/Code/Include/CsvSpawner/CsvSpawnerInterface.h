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

namespace CsvSpawner
{
    class CsvSpawnerInterface : public AZ::EBusTraits
    {
    public:
        virtual ~CsvSpawnerInterface() = default;

        virtual void OnEntitiesSpawnBegin(const AZStd::string& physicsSceneName, const AZ::EntityId& parentId) = 0;

        virtual void OnEntitiesSpawnFinished(const AZStd::string& physicsSceneName, const AZ::EntityId& parentId) = 0;

        // EBus Configuration - allow multiple listeners
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
    };

    // Create an EBus using the notification interface
    using CsvSpawnerNotificationBus = AZ::EBus<CsvSpawnerInterface>;

} // namespace CsvSpawner
