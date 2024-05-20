/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace LevelModificationTools
{
    //! Spawn a prefab in the level, it sets the parent of the spawned entities to the parentId and returns a ticket to track the spawned
    //! entities
    //! @param spawnableAsset The asset of the prefab to spawn
    //! @param parentId The parent entity id of the spawned entities
    //! @return The spawn ticket to track the spawned entities
    AzFramework::EntitySpawnTicket SpawnPrefab(const AZ::Data::Asset<AzFramework::Spawnable>& spawnableAsset, const AZ::EntityId parentId);
} // namespace LevelModificationTools
