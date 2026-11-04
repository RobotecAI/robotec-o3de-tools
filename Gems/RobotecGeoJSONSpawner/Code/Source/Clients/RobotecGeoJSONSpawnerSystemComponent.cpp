/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotecGeoJSONSpawnerSystemComponent.h"

#include <RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecGeoJSONSpawner
{
    AZ_COMPONENT_IMPL(
        RobotecGeoJSONSpawnerSystemComponent, "RobotecGeoJSONSpawnerSystemComponent", RobotecGeoJSONSpawnerSystemComponentTypeId);

    void RobotecGeoJSONSpawnerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecGeoJSONSpawnerSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void RobotecGeoJSONSpawnerSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerService"));
    }

    void RobotecGeoJSONSpawnerSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerService"));
    }

    void RobotecGeoJSONSpawnerSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotecGeoJSONSpawnerSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void RobotecGeoJSONSpawnerSystemComponent::Init()
    {
    }

    void RobotecGeoJSONSpawnerSystemComponent::Activate()
    {
    }

    void RobotecGeoJSONSpawnerSystemComponent::Deactivate()
    {
    }
} // namespace RobotecGeoJSONSpawner
