/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerComponent.h"

#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <rapidjson/schema.h>

namespace GeoJSONSpawner
{
    GeoJSONSpawnerComponent::GeoJSONSpawnerComponent(const GeoJSONUtils::GeoJSONSpawnerConfiguration& configuration)
        : m_configuration(configuration)
    {
    }

    void GeoJSONSpawnerComponent::Activate()
    {
        GeoJSONSpawnerRequestBus::Handler::BusConnect(GetEntityId());
    }

    void GeoJSONSpawnerComponent::Deactivate()
    {
        GeoJSONSpawnerRequestBus::Handler::BusDisconnect();
    }

    void GeoJSONSpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        GeoJSONUtils::GeoJSONSpawnerConfiguration::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerComponent, AZ::Component>()->Version(0)->Field(
                "Configuration", &GeoJSONSpawnerComponent::m_configuration);
        }
    }

    void GeoJSONSpawnerComponent::Spawn(const AZStd::string& rawJsonString)
    {
        const auto result = GeoJSONUtils::ParseJSONFromRawString(rawJsonString);
        m_spawnableTickets = GeoJSONUtils::SpawnEntities(result, m_configuration.m_spawnableAssets, GetEntityId());
    }
} // namespace GeoJSONSpawner
