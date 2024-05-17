/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabVariantComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/TransformComponent.h>
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
            SpawnPrefab(m_config.m_defaultPrefabVariant);
        }
        LevelModificationToolsRequestBus::Handler::BusConnect(m_config.m_groupId);
    }

    void PrefabVariantComponent::Deactivate()
    {
        LevelModificationToolsRequestBus::Handler::BusDisconnect();
    }


    void PrefabVariantComponent::SpawnPrefab( AZ::Data::Asset<AzFramework::Spawnable> prefabAsset)
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

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
        optionalArgs.m_completionCallback = [this](auto ticket, auto result)
        {
            if (!result.empty())
            {
                const AZ::Entity* root = *result.begin();
                auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
                transformInterface->SetParent(GetEntityId());
            }
        };

        optionalArgs.m_priority = AzFramework::SpawnablePriority_Low;
        AzFramework::EntitySpawnTicket ticket(prefabAsset);
        spawner->SpawnAllEntities(ticket, optionalArgs);
        m_spawnTicket = ticket;
    }

    void PrefabVariantComponent::SetPrefabVariant(AZ::s32 variantId)
    {
        if (variantId < 0)
        {
            m_spawnTicket = AzFramework::EntitySpawnTicket();
        }

        const auto assetIter = m_config.m_prefabVariants.find(variantId);
        AZ_Warning("PrefabVariantComponent", assetIter != m_config.m_prefabVariants.end(), "Prefab variant with id %d not found.", variantId);
        if (assetIter != m_config.m_prefabVariants.end())
        {
            SpawnPrefab(assetIter->second);
        }
    }

    void PrefabVariantComponent::Reflect(AZ::ReflectContext* context)
    {
        LevelModificationToolsRequests::Reflect(context);
        PrefabVariantConfig::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PrefabVariantComponent, AZ::Component>()
                ->Version(1)
                ->Field("Config", &PrefabVariantComponent::m_config);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PrefabVariantComponent>("Prefab Variant Component", "A component that manages prefab variants.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "LevelModificationTools")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrefabVariantComponent::m_config,
                        "Config", "The configuration for the component.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }
} // namespace LevelModificationTools