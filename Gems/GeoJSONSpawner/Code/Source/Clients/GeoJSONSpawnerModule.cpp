/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerSystemComponent.h"
#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>
#include <GeoJSONSpawnerModuleInterface.h>

namespace GeoJSONSpawner
{
    class GeoJSONSpawnerModule : public GeoJSONSpawnerModuleInterface
    {
    public:
        AZ_RTTI(GeoJSONSpawnerModule, GeoJSONSpawnerModuleTypeId, GeoJSONSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoJSONSpawnerModule, AZ::SystemAllocator);
    };
} // namespace GeoJSONSpawner

AZ_DECLARE_MODULE_CLASS(Gem_GeoJSONSpawner, GeoJSONSpawner::GeoJSONSpawnerModule)
