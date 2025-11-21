/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "RobotecGeoJSONSpawnerModuleInterface.h"

#include "ROS2/ROS2Bus.h"
#include "RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerComponent.h"

#include <AzCore/Memory/Memory.h>

#include <RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerTypeIds.h>

#include <Clients/RobotecGeoJSONSpawnerSystemComponent.h>

namespace RobotecGeoJSONSpawner
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        RobotecGeoJSONSpawnerModuleInterface, "RobotecGeoJSONSpawnerModuleInterface", RobotecGeoJSONSpawnerModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(RobotecGeoJSONSpawnerModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(RobotecGeoJSONSpawnerModuleInterface, AZ::SystemAllocator);

    RobotecGeoJSONSpawnerModuleInterface::RobotecGeoJSONSpawnerModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            { RobotecGeoJSONSpawnerSystemComponent::CreateDescriptor(), RobotecGeoJSONSpawnerComponent::CreateDescriptor() });
    }

    AZ::ComponentTypeList RobotecGeoJSONSpawnerModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<RobotecGeoJSONSpawnerSystemComponent>(),
        };
    }
} // namespace RobotecGeoJSONSpawner
