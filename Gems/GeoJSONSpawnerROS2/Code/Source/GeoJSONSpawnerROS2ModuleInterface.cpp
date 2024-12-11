/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2ModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <GeoJSONSpawnerROS2/GeoJSONSpawnerROS2TypeIds.h>

#include <Clients/GeoJSONSpawnerROS2SystemComponent.h>
#include <ROS2/GeoJSONSpawnerROS2.h>

namespace GeoJSONSpawnerROS2
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        GeoJSONSpawnerROS2ModuleInterface, "GeoJSONSpawnerROS2ModuleInterface", GeoJSONSpawnerROS2ModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(GeoJSONSpawnerROS2ModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(GeoJSONSpawnerROS2ModuleInterface, AZ::SystemAllocator);

    GeoJSONSpawnerROS2ModuleInterface::GeoJSONSpawnerROS2ModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(), { GeoJSONSpawnerROS2SystemComponent::CreateDescriptor(), GeoJSONSpawnerROS2::CreateDescriptor() });
    }

    AZ::ComponentTypeList GeoJSONSpawnerROS2ModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<GeoJSONSpawnerROS2SystemComponent>(),
        };
    }
} // namespace GeoJSONSpawnerROS2
