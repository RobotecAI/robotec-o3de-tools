/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2SystemComponent.h"

#include <GeoJSONSpawnerROS2/GeoJSONSpawnerROS2TypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace GeoJSONSpawnerROS2
{
    AZ_COMPONENT_IMPL(GeoJSONSpawnerROS2SystemComponent, "GeoJSONSpawnerROS2SystemComponent", GeoJSONSpawnerROS2SystemComponentTypeId);

    void GeoJSONSpawnerROS2SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerROS2SystemComponent, AZ::Component>()->Version(0);
        }
    }

    void GeoJSONSpawnerROS2SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GeoJSONSpawnerROS2Service"));
    }

    void GeoJSONSpawnerROS2SystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GeoJSONSpawnerROS2Service"));
    }

    void GeoJSONSpawnerROS2SystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void GeoJSONSpawnerROS2SystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void GeoJSONSpawnerROS2SystemComponent::Init()
    {
    }

    void GeoJSONSpawnerROS2SystemComponent::Activate()
    {
    }

    void GeoJSONSpawnerROS2SystemComponent::Deactivate()
    {
    }

} // namespace GeoJSONSpawnerROS2
