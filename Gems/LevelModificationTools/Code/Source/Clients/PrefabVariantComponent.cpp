/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabVariantComponent.h"

#include <AzCore/Serialization/EditContext.h>

#include "SpawnPrefab.h"
#include <LevelModificationTools/PrefabVariantRequestsBus.h>

#include <utility>

namespace LevelModificationTools
{
    PrefabVariantComponent::PrefabVariantComponent(PrefabVariantConfig config)
        : m_config(std::move(config))
    {
    }
    void PrefabVariantComponent::Activate()
    {
        if (m_config.m_defaultPrefabVariant)
        {
            m_spawnTicket = SpawnPrefab(m_config.m_defaultPrefabVariant, GetEntityId());
        }
        PrefabVariantRequestsBus::Handler::BusConnect(m_config.m_groupId);
    }

    void PrefabVariantComponent::Deactivate()
    {
        PrefabVariantRequestsBus::Handler::BusDisconnect();
    }

    void PrefabVariantComponent::SetPrefabVariant(AZ::s32 variantId)
    {
        if (variantId < 0 && variantId >= m_config.m_prefabVariants.size())
        {
            m_spawnTicket = AzFramework::EntitySpawnTicket();
            return;
        }

        const auto assetIter = m_config.m_prefabVariants.begin() + variantId;
        AZ_Warning(
            "PrefabVariantComponent", assetIter != m_config.m_prefabVariants.end(), "Prefab variant with id %d not found.", variantId);
        if (assetIter != m_config.m_prefabVariants.end())
        {
            m_spawnTicket = SpawnPrefab(*assetIter, GetEntityId());
        }
    }

    void PrefabVariantComponent::Reflect(AZ::ReflectContext* context)
    {
        PrefabVariantRequests::Reflect(context);
        PrefabVariantConfig::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PrefabVariantComponent, AZ::Component>()->Version(1)->Field(
                "Config", &PrefabVariantComponent::m_config);
        }
    }
} // namespace LevelModificationTools
