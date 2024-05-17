/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SpawnPrefab.h"
#include <AzFramework/Components/TransformComponent.h>

namespace LevelModificationTools
{

    AzFramework::EntitySpawnTicket SpawnPrefab(const AZ::Data::Asset<AzFramework::Spawnable>& spawnableAsset, const AZ::EntityId parentId)
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, parentId, &AZ::TransformBus::Events::GetWorldTM);

        // Spawn the prefab variant
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "SpawnableEntitiesDefinition not found.");

        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

        optionalArgs.m_preInsertionCallback = [transform](auto id, auto view)
        {
            AZ::Entity* root = *view.begin();
            auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
            transformInterface->SetWorldTM(transform);
        };
        optionalArgs.m_completionCallback = [parentId](auto ticket, auto result)
        {
            if (!result.empty())
            {
                const AZ::Entity* root = *result.begin();
                auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
                transformInterface->SetParent(parentId);
            }
        };

        optionalArgs.m_priority = AzFramework::SpawnablePriority_Low;
        AzFramework::EntitySpawnTicket ticket(spawnableAsset);
        spawner->SpawnAllEntities(ticket, optionalArgs);
        return ticket;
    }
} // namespace LevelModificationTools
