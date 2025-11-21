/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
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
