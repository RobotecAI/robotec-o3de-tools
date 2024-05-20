
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <LevelModificationTools/LevelModificationToolsTypeIds.h>
#include <LevelModificationTools/PrefabVariantRequestsBus.h>

namespace LevelModificationTools
{
    struct PrefabVariantConfig
    {
        AZ_TYPE_INFO(PrefabVariantConfig, PrefabVariantConfigTypeId);
        static void Reflect(AZ::ReflectContext* context);

        AZ::s32 m_groupId = 0; //! Region id for the spawned entities
        AZStd::unordered_map<AZ::s32, AZ::Data::Asset<AzFramework::Spawnable>> m_prefabVariants;
        AZ::Data::Asset<AzFramework::Spawnable> m_defaultPrefabVariant;
    };
} // namespace LevelModificationTools
