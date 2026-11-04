/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotecGeoJSONSpawnerSystemComponent.h"
#include <RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerTypeIds.h>
#include <RobotecGeoJSONSpawnerModuleInterface.h>

namespace RobotecGeoJSONSpawner
{
    class RobotecGeoJSONSpawnerModule : public RobotecGeoJSONSpawnerModuleInterface
    {
    public:
        AZ_RTTI(RobotecGeoJSONSpawnerModule, RobotecGeoJSONSpawnerModuleTypeId, RobotecGeoJSONSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecGeoJSONSpawnerModule, AZ::SystemAllocator);
    };
} // namespace RobotecGeoJSONSpawner

AZ_DECLARE_MODULE_CLASS(Gem_RobotecGeoJSONSpawner, RobotecGeoJSONSpawner::RobotecGeoJSONSpawnerModule)
