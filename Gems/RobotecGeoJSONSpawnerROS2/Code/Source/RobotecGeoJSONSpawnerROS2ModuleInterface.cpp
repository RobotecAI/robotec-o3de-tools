/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
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
