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
#include <LevelModificationTools/LevelModificationToolsBus.h>

namespace LevelModificationTools
{
    PrefabVariantComponent::PrefabVariantComponent(const LevelModificationTools::PrefabVariantConfig& config)
        : m_config(config)
    {
    }
    void PrefabVariantComponent::Activate()
    {
        if (m_config.m_defaultPrefabVariant)
        {
            m_spawnTicket = SpawnPrefab(m_config.m_defaultPrefabVariant, GetEntityId());
        }
        LevelModificationToolsRequestBus::Handler::BusConnect(m_config.m_groupId);
    }

    void PrefabVariantComponent::Deactivate()
    {
        LevelModificationToolsRequestBus::Handler::BusDisconnect();
    }

    void PrefabVariantComponent::SetPrefabVariant(AZ::s32 variantId)
    {
        if (variantId < 0)
        {
            m_spawnTicket = AzFramework::EntitySpawnTicket();
        }

        const auto assetIter = m_config.m_prefabVariants.find(variantId);
        AZ_Warning(
            "PrefabVariantComponent", assetIter != m_config.m_prefabVariants.end(), "Prefab variant with id %d not found.", variantId);
        if (assetIter != m_config.m_prefabVariants.end())
        {
            m_spawnTicket = SpawnPrefab(assetIter->second, GetEntityId());
        }
    }

    void PrefabVariantComponent::Reflect(AZ::ReflectContext* context)
    {
        LevelModificationToolsRequests::Reflect(context);
        PrefabVariantConfig::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PrefabVariantComponent, AZ::Component>()->Version(1)->Field(
                "Config", &PrefabVariantComponent::m_config);
        }
    }
} // namespace LevelModificationTools
