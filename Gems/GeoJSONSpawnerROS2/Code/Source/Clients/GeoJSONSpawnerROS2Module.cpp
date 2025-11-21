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
#include <RobotecGeoJSONSpawnerROS2ModuleInterface.h>

namespace RobotecGeoJSONSpawnerROS2
{
    class RobotecGeoJSONSpawnerROS2Module : public RobotecGeoJSONSpawnerROS2ModuleInterface
    {
    public:
        AZ_RTTI(RobotecGeoJSONSpawnerROS2Module, RobotecGeoJSONSpawnerROS2ModuleTypeId, RobotecGeoJSONSpawnerROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecGeoJSONSpawnerROS2Module, AZ::SystemAllocator);
    };
} // namespace RobotecGeoJSONSpawnerROS2

AZ_DECLARE_MODULE_CLASS(Gem_RobotecGeoJSONSpawnerROS2, RobotecGeoJSONSpawnerROS2::RobotecGeoJSONSpawnerROS2Module)
