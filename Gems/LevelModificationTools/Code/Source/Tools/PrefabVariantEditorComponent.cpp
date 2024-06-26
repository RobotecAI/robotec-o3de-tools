
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabVariantEditorComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <Clients/PrefabVariantComponent.h>
#include <Clients/SpawnPrefab.h>
namespace LevelModificationTools
{
    void PrefabVariantEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PrefabVariantEditorComponent, AZ::Component>()
                ->Version(1)
                ->Field("Config", &PrefabVariantEditorComponent::m_config)
                ->Field("VariantToPeek", &PrefabVariantEditorComponent::m_variantToPeek);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<PrefabVariantEditorComponent>("Prefab Variant Editor Component", "A component that manages prefab variants.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "LevelModificationTools")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrefabVariantEditorComponent::m_config,
                        "Config",
                        "The configuration for the component.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PrefabVariantEditorComponent::OnConfigChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrefabVariantEditorComponent::m_variantToPeek,
                        "Variant ID to preview in the Editor",
                        "The group id of the to take a peek at in the editor.")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Preview variant", "Preview the given variant.")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Preview variant")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PrefabVariantEditorComponent::PreviewVariant);
            }
        }
    }

    void PrefabVariantEditorComponent::Activate()
    {
        PrefabVariantRequestsBus::Handler::BusConnect(m_config.m_groupId);
        if (m_config.m_defaultPrefabVariant)
        {
            m_config.m_defaultPrefabVariant.QueueLoad();
            m_spawnTicket = SpawnPrefab(m_config.m_defaultPrefabVariant, GetEntityId());
        }
    }

    void PrefabVariantEditorComponent::Deactivate()
    {
        m_spawnTicket = AzFramework::EntitySpawnTicket();
        PrefabVariantRequestsBus::Handler::BusDisconnect();
    }

    void PrefabVariantEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        m_spawnTicket = AzFramework::EntitySpawnTicket();
        gameEntity->CreateComponent<PrefabVariantComponent>(m_config);
    }

    AZ::Crc32 PrefabVariantEditorComponent::OnConfigChanged()
    {
        if (m_config.m_defaultPrefabVariant)
        {
            m_config.m_defaultPrefabVariant.QueueLoad();
            m_spawnTicket = SpawnPrefab(m_config.m_defaultPrefabVariant, GetEntityId());
        }
        if (PrefabVariantRequestsBus::Handler::BusIsConnected())
        {
            PrefabVariantRequestsBus::Handler::BusDisconnect();
            PrefabVariantRequestsBus::Handler::BusConnect(m_config.m_groupId);
        }
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZ::Crc32 PrefabVariantEditorComponent::PreviewVariant()
    {
        PrefabVariantRequestsBus::Event(m_config.m_groupId, &PrefabVariantRequests::SetPrefabVariant, m_variantToPeek);
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    // LevelModificationToolsRequestBus::Handler overrides
    void PrefabVariantEditorComponent::SetPrefabVariant(AZ::s32 variantId)
    {
        if (variantId >= 0 && variantId < m_config.m_prefabVariants.size())
        {
            m_config.m_prefabVariants[variantId].QueueLoad();
            m_spawnTicket = SpawnPrefab(m_config.m_prefabVariants[variantId], GetEntityId());
            return;
        }
        m_spawnTicket = AzFramework::EntitySpawnTicket();
    }

} // namespace LevelModificationTools
