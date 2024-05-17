/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabVariantConfig.h"
#include <AzCore/Serialization/EditContext.h>
#include <LevelModificationTools/LevelModificationToolsTypeIds.h>
#include <AzCore/Asset/AssetSerializer.h>

namespace LevelModificationTools
{
    void PrefabVariantConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PrefabVariantConfig>()
                ->Version(1)
                ->Field("PrefabVariants", &PrefabVariantConfig::m_prefabVariants)
                ->Field("GroupId", &PrefabVariantConfig::m_groupId)
                ->Field("DefaultPrefabVariant", &PrefabVariantConfig::m_defaultPrefabVariant);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PrefabVariantConfig>("Prefab Variant Component", "A component that manages prefab variants.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "LevelModificationTools")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrefabVariantConfig::m_groupId,
                        "Group Id",
                        "The region id for the spawned entities.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrefabVariantConfig::m_defaultPrefabVariant,
                        "Default Prefab Variant",
                        "The default prefab variant to spawn.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrefabVariantConfig::m_prefabVariants,
                        "Prefab Variants",
                        "A map of prefab variant ids to spawnable assets.");
            }
        }
    }
} // namespace LevelModificationTools
