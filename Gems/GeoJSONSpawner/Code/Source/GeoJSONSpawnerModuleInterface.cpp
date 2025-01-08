/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerModuleInterface.h"

#include "GeoJSONSpawner/GeoJSONSpawnerComponent.h"
#include "ROS2/ROS2Bus.h"

#include <AzCore/Memory/Memory.h>

#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>

#include <Clients/GeoJSONSpawnerSystemComponent.h>

namespace GeoJSONSpawner
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(GeoJSONSpawnerModuleInterface, "GeoJSONSpawnerModuleInterface", GeoJSONSpawnerModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(GeoJSONSpawnerModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(GeoJSONSpawnerModuleInterface, AZ::SystemAllocator);

    GeoJSONSpawnerModuleInterface::GeoJSONSpawnerModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(), { GeoJSONSpawnerSystemComponent::CreateDescriptor(), GeoJSONSpawnerComponent::CreateDescriptor() });
    }

    AZ::ComponentTypeList GeoJSONSpawnerModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<GeoJSONSpawnerSystemComponent>(),
        };
    }
} // namespace GeoJSONSpawner
