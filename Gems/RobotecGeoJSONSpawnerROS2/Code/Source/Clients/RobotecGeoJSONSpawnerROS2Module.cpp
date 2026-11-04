/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
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
