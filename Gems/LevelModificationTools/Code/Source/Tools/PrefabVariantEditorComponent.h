/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Clients/PrefabVariantConfig.h>
#include <LevelModificationTools/LevelModificationToolsTypeIds.h>
#include <LevelModificationTools/PrefabVariantRequestsBus.h>

namespace LevelModificationTools
{
    class PrefabVariantEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private PrefabVariantRequestsBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(PrefabVariantEditorComponent, "{018f8675-2a33-7855-a242-a45e6ec7fd4b}");
        PrefabVariantEditorComponent() = default;
        ~PrefabVariantEditorComponent() override = default;

        // Component overrides
        void Activate() override;
        void Deactivate() override;

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        AZ::Crc32 OnConfigChanged();
        AZ::Crc32 PreviewVariant();
        // LevelModificationToolsRequestBus::Handler overrides
        void SetPrefabVariant(AZ::s32 variantId) override;

        AzFramework::EntitySpawnTicket m_spawnTicket; //! Currently spawned ticket
        PrefabVariantConfig m_config; //! Configuration for the component
        AZ::s32 m_variantToPeek = 0; //! The group id of the to take a peek at in the editor
    };
} // namespace LevelModificationTools
