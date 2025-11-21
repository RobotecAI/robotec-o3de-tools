/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
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
