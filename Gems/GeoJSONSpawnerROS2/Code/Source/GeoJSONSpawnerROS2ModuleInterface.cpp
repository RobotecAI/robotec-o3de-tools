/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "RobotecGeoJSONSpawnerROS2ModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <RobotecGeoJSONSpawnerROS2/RobotecGeoJSONSpawnerROS2TypeIds.h>

#include <Clients/RobotecGeoJSONSpawnerROS2SystemComponent.h>
#include <ROS2/RobotecGeoJSONSpawnerROS2.h>

namespace RobotecGeoJSONSpawnerROS2
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        RobotecGeoJSONSpawnerROS2ModuleInterface,
        "RobotecGeoJSONSpawnerROS2ModuleInterface",
        RobotecGeoJSONSpawnerROS2ModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(RobotecGeoJSONSpawnerROS2ModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(RobotecGeoJSONSpawnerROS2ModuleInterface, AZ::SystemAllocator);

    RobotecGeoJSONSpawnerROS2ModuleInterface::RobotecGeoJSONSpawnerROS2ModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            { RobotecGeoJSONSpawnerROS2SystemComponent::CreateDescriptor(), RobotecGeoJSONSpawnerROS2::CreateDescriptor() });
    }

    AZ::ComponentTypeList RobotecGeoJSONSpawnerROS2ModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<RobotecGeoJSONSpawnerROS2SystemComponent>(),
        };
    }
} // namespace RobotecGeoJSONSpawnerROS2
