/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerSystemComponent.h"

#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace GeoJSONSpawner
{
    AZ_COMPONENT_IMPL(GeoJSONSpawnerSystemComponent, "GeoJSONSpawnerSystemComponent", GeoJSONSpawnerSystemComponentTypeId);

    void GeoJSONSpawnerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void GeoJSONSpawnerSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GeoJSONSpawnerService"));
    }

    void GeoJSONSpawnerSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GeoJSONSpawnerService"));
    }

    void GeoJSONSpawnerSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void GeoJSONSpawnerSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void GeoJSONSpawnerSystemComponent::Init()
    {
    }

    void GeoJSONSpawnerSystemComponent::Activate()
    {
    }

    void GeoJSONSpawnerSystemComponent::Deactivate()
    {
    }
} // namespace GeoJSONSpawner
