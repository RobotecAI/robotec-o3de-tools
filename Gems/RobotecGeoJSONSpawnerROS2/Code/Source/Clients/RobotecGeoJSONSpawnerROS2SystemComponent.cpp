/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotecGeoJSONSpawnerROS2SystemComponent.h"

#include <RobotecGeoJSONSpawnerROS2/RobotecGeoJSONSpawnerROS2TypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace RobotecGeoJSONSpawnerROS2
{
    AZ_COMPONENT_IMPL(
        RobotecGeoJSONSpawnerROS2SystemComponent,
        "RobotecGeoJSONSpawnerROS2SystemComponent",
        RobotecGeoJSONSpawnerROS2SystemComponentTypeId);

    void RobotecGeoJSONSpawnerROS2SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecGeoJSONSpawnerROS2SystemComponent, AZ::Component>()->Version(0);
        }
    }

    void RobotecGeoJSONSpawnerROS2SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerROS2Service"));
    }

    void RobotecGeoJSONSpawnerROS2SystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerROS2Service"));
    }

    void RobotecGeoJSONSpawnerROS2SystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotecGeoJSONSpawnerROS2SystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void RobotecGeoJSONSpawnerROS2SystemComponent::Init()
    {
    }

    void RobotecGeoJSONSpawnerROS2SystemComponent::Activate()
    {
    }

    void RobotecGeoJSONSpawnerROS2SystemComponent::Deactivate()
    {
    }

} // namespace RobotecGeoJSONSpawnerROS2
