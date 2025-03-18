/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with
 * the permission of the copyright holders.  If you encounter this file and do
 * not have permission, please contact the copyright holders and delete this
 * file.
 */

#include "CsvSpawnerUtils.h"

#include <AzFramework/Physics/CollisionBus.h>
#include <CsvSpawner/CsvSpawnerInterface.h>

#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <cstdlib>
#include <random>

namespace CsvSpawner::CsvSpawnerUtils
{

    void CsvSpawnableEntityInfo::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<CsvSpawnableEntityInfo>()
                ->Version(1)
                ->Field("Id", &CsvSpawnableEntityInfo::m_id)
                ->Field("Transform", &CsvSpawnableEntityInfo::m_transform)
                ->Field("Name", &CsvSpawnableEntityInfo::m_name)
                ->Field("Seed", &CsvSpawnableEntityInfo::m_seed);
        }
    }
    void CsvSpawnableAssetConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<CsvSpawnableAssetConfiguration>()
                ->Version(1)
                ->Field("Name", &CsvSpawnableAssetConfiguration::m_name)
                ->Field("Spawnable", &CsvSpawnableAssetConfiguration::m_spawnables)
                ->Field("PositionStdDev", &CsvSpawnableAssetConfiguration::m_positionStdDev)
                ->Field("RotationStdDev", &CsvSpawnableAssetConfiguration::m_rotationStdDev)
                ->Field("ScaleStdDev", &CsvSpawnableAssetConfiguration::m_scaleStdDev)
                ->Field("PlaceOnTerrain", &CsvSpawnableAssetConfiguration::m_placeOnTerrain)
                ->Field("CollisionGroup", &CsvSpawnableAssetConfiguration::m_selectedCollisionGroup);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<CsvSpawnableAssetConfiguration>("SpawnableAssetConfiguration", "SpawnableAssetConfiguration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "SpawnableAssetConfiguration")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "CsvSpawner")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &CsvSpawnableAssetConfiguration::m_name, "Name", "Name of the spawnable field")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &CsvSpawnableAssetConfiguration::m_spawnables, "Spawnables", "Vector of spawnables")
                    ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.spawnable")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnableAssetConfiguration::m_positionStdDev,
                        "Position Std. Dev.",
                        "Position standard deviation, in meters")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnableAssetConfiguration::m_rotationStdDev,
                        "Rotation Std. Dev. ",
                        "Rotation standard deviation, in degrees")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnableAssetConfiguration::m_placeOnTerrain,
                        "Place on Terrain",
                        "Perform scene query raytrace to place on terrain")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &CsvSpawnableAssetConfiguration::OnPlaceOnTerrainChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnableAssetConfiguration::m_scaleStdDev,
                        "Scale Std. Dev.",
                        "Scale standard deviation, in meters")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CsvSpawnableAssetConfiguration::m_selectedCollisionGroup,
                        "Collision Group",
                        "To which collision group this target will be attached")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, &CsvSpawnableAssetConfiguration::IsCollisionLayerEnabled);
            }
        }
    }

    bool CsvSpawnableAssetConfiguration::IsCollisionLayerEnabled() const
    {
        return !m_placeOnTerrain;
    }

    AZ::Crc32 CsvSpawnableAssetConfiguration::OnPlaceOnTerrainChanged()
    {
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration> GetSpawnableAssetFromVector(
        AZStd::vector<CsvSpawnableAssetConfiguration> spawnableAssetConfigurations)
    {
        AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration> spawnableAssetConfiguration;
        for (auto& spawnableAssetConfigurationElement : spawnableAssetConfigurations)
        {
            spawnableAssetConfiguration[spawnableAssetConfigurationElement.m_name] = spawnableAssetConfigurationElement;
        }
        return spawnableAssetConfiguration;
    }

    template<typename T>
    T GetRandomElement(const AZStd::vector<T>& vec, std::mt19937& gen)
    {
        std::uniform_int_distribution<int> distribution(0, vec.size() - 1);
        return vec.at(distribution(gen));
    }

    AZ::Transform GetRandomTransform(
        const AZ::Vector3& stdDevTranslation, const AZ::Vector3& stdDevRotation, float stdDevScale, std::mt19937& gen)
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZStd::vector<std::normal_distribution<float>> distributions;
        distributions.emplace_back(0.0f, stdDevTranslation.GetX());
        distributions.emplace_back(0.0f, stdDevTranslation.GetY());
        distributions.emplace_back(0.0f, stdDevTranslation.GetZ());
        distributions.emplace_back(0.0f, stdDevRotation.GetX());
        distributions.emplace_back(0.0f, stdDevRotation.GetY());
        distributions.emplace_back(0.0f, stdDevRotation.GetZ());
        distributions.emplace_back(1.0f, stdDevScale);

        transform.SetTranslation(AZ::Vector3(distributions[0](gen), distributions[1](gen), distributions[2](gen)));
        const AZ::Quaternion rotation =
            AZ::Quaternion::CreateFromEulerAnglesDegrees(AZ::Vector3(distributions[3](gen), distributions[4](gen), distributions[5](gen)));
        transform.SetRotation(rotation);
        transform.SetUniformScale(AZStd::max(0.0f, distributions[6](gen)));
        return transform;
    }

    AZStd::optional<AZ::Vector3> RaytraceTerrain(
        const AZ::Vector3& location,
        const AzPhysics::SceneHandle sceneHandle,
        const AZ::Vector3& gravityDirection,
        float maxDistance,
        const AzPhysics::CollisionGroup& collisionGroup)
    {
        AZStd::optional<AZ::Vector3> hitPosition = AZStd::nullopt;

        AZ_Assert(sceneHandle != AzPhysics::InvalidSceneHandle, "Unable to get  scene handle");
        if (sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return hitPosition;
        }

        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(physicsSystem, "Unable to get physics system interface");
        AZ_Assert(sceneInterface, "Unable to get physics scene interface");

        if (!sceneInterface || !physicsSystem)
        {
            return hitPosition;
        }

        AzPhysics::RayCastRequest request;
        request.m_start = location;
        request.m_direction = gravityDirection;
        request.m_distance = maxDistance;
        request.m_collisionGroup = collisionGroup;

        AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(sceneHandle, &request);

        if (!result.m_hits.empty())
        {
            hitPosition = result.m_hits.front().m_position;
        }

        return hitPosition;
    }

    AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> SpawnEntities(
        const AZStd::vector<CsvSpawnableEntityInfo>& entitiesToSpawn,
        const AZStd::unordered_map<AZStd::string, CsvSpawnableAssetConfiguration>& spawnableAssetConfiguration,
        AZ::u64 defaultSeed,
        const AZStd::string& physicsSceneName,
        AZ::EntityId parentId)
    {
        SpawnInfo broadcastSpawnInfo =
            SpawnInfo{ entitiesToSpawn, physicsSceneName, parentId }; // Spawn Info used in CsvSpawner EBus notify.
        SpawnStatusCode spawnStatusCode; // Spawn Status Code Status used for CsvSpawner EBus notify - OnEntitiesSpawnFinished.

        // Call CsvSpawner EBus notification - Begin
        CsvSpawnerNotificationBus::Broadcast(&CsvSpawnerInterface::OnEntitiesSpawnBegin, broadcastSpawnInfo);

        auto sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Unable to get physics scene interface");
        const auto sceneHandle = sceneInterface->GetSceneHandle(physicsSceneName);

        AZStd::unordered_map<int, AzFramework::EntitySpawnTicket> tickets{};
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawnable entities definition");

        // get parent transform
        AZ::Transform parentTransform = AZ::Transform::CreateIdentity();
        if (parentId.IsValid())
        {
            AZ::TransformBus::EventResult(parentTransform, parentId, &AZ::TransformBus::Events::GetWorldTM);
        }

        for (auto& spawnables : spawnableAssetConfiguration)
        {
            for (auto spawnable : spawnables.second.m_spawnables)
            {
                spawnable.QueueLoad();
            }
        }

        for (const auto& entityConfig : entitiesToSpawn)
        {
            if (!spawnableAssetConfiguration.contains(entityConfig.m_name))
            {
                AZ_Error("CsvSpawner", false, "SpawnableAssetConfiguration %s not found", entityConfig.m_name.c_str());

                // Add notify code status
                spawnStatusCode |= SpawnStatusCode::ErrorOccurred;
                continue;
            }

            const auto& spawnConfig = spawnableAssetConfiguration.at(entityConfig.m_name);

            // if seed is not set for this row, use the default seed
            std::mt19937 gen = entityConfig.m_seed > 0 ? std::mt19937(entityConfig.m_seed) : std::mt19937(defaultSeed++);

            const auto& spawnable = GetRandomElement(spawnConfig.m_spawnables, gen);

            AZ::Transform transform = parentTransform * entityConfig.m_transform *
                GetRandomTransform(spawnConfig.m_positionStdDev, spawnConfig.m_rotationStdDev, spawnConfig.m_scaleStdDev, gen);

            if (spawnConfig.m_placeOnTerrain)
            {
                // Get collision group chosen from editor
                AzPhysics::CollisionGroup collisionGroup;
                Physics::CollisionRequestBus::BroadcastResult(
                    collisionGroup, &Physics::CollisionRequests::GetCollisionGroupById, spawnConfig.m_selectedCollisionGroup);

                const AZStd::optional<AZ::Vector3> hitPosition =
                    RaytraceTerrain(transform.GetTranslation(), sceneHandle, -AZ::Vector3::CreateAxisZ(), 1000.0f, collisionGroup);

                if (hitPosition.has_value())
                {
                    transform.SetTranslation(hitPosition.value());
                }
                else
                {
                    // Add notify code status
                    spawnStatusCode |= SpawnStatusCode::ErrorOccurred;

                    continue; // Skip this entity if we can't find a valid position and
                              // place on terrain is enabled.
                }
            }
            AZ_Assert(spawner, "Unable to get spawnable entities definition");
            AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
            AzFramework::EntitySpawnTicket ticket(spawnable);
            // Set the pre-spawn callback to set the name of the root entity to the name
            // of the spawnable
            optionalArgs.m_preInsertionCallback = [transform, &spawnStatusCode](auto id, auto view)
            {
                if (view.empty())
                {
                    // Add notify code status
                    spawnStatusCode |= SpawnStatusCode::ErrorOccurred;

                    return;
                }
                AZ::Entity* root = *view.begin();
                AZ_Assert(root, "Invalid root entity");

                auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
                transformInterface->SetWorldTM(transform);
            };
            optionalArgs.m_completionCallback =
                [parentId, &spawnStatusCode](
                    [[maybe_unused]] AzFramework::EntitySpawnTicket::Id ticketId, AzFramework::SpawnableConstEntityContainerView view)
            {
                if (view.empty())
                {
                    // Add notify code status
                    spawnStatusCode |= SpawnStatusCode::ErrorOccurred;

                    return;
                }
                const AZ::Entity* root = *view.begin();
                AZ::TransformBus::Event(root->GetId(), &AZ::TransformBus::Events::SetParent, parentId);
            };
            optionalArgs.m_priority = AzFramework::SpawnablePriority_Lowest;
            spawner->SpawnAllEntities(ticket, optionalArgs);
            tickets[entityConfig.m_id] = AZStd::move(ticket);
        }

        // Check is success spawn
        tickets.empty() ? spawnStatusCode |= SpawnStatusCode::Fail : spawnStatusCode |= SpawnStatusCode::Success;
        // Call CsvSpawner EBus notification - Finished
        CsvSpawnerNotificationBus::Broadcast(&CsvSpawnerInterface::OnEntitiesSpawnFinished, broadcastSpawnInfo, spawnStatusCode);

        return tickets;
    }

} // namespace CsvSpawner::CsvSpawnerUtils
