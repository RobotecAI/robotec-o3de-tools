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
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <LevelModificationTools/LevelModificationToolsBus.h>
#include <LevelModificationTools/LevelModificationToolsTypeIds.h>
#include "PrefabVariantConfig.h"

namespace LevelModificationTools
{
    class PrefabVariantComponent
        : public AZ::Component
        , private LevelModificationToolsRequestBus::Handler
    {
    public:
        AZ_COMPONENT(PrefabVariantComponent, "{018f8611-85fc-7e0f-a879-b3a0ff793535}", AZ::Component);
        PrefabVariantComponent() = default;
        PrefabVariantComponent(const PrefabVariantConfig& config);
        ~PrefabVariantComponent() override = default;


        // Component overrides
        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        // LevelModificationToolsRequestBus::Handler overrides
        void SetPrefabVariant(AZ::s32 variantId) override;

        void SpawnPrefab( AZ::Data::Asset<AzFramework::Spawnable> prefabAsset);

        AzFramework::EntitySpawnTicket m_spawnTicket; //! Currently spawned ticket
        PrefabVariantConfig m_config; //! Configuration for the component

    };
} // namespace LevelModificationTools