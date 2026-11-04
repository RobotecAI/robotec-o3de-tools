/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
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
