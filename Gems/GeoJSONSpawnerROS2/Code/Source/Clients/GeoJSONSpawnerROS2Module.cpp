/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2SystemComponent.h"
#include <GeoJSONSpawnerROS2/GeoJSONSpawnerROS2TypeIds.h>
#include <GeoJSONSpawnerROS2ModuleInterface.h>

namespace GeoJSONSpawnerROS2
{
    class GeoJSONSpawnerROS2Module : public GeoJSONSpawnerROS2ModuleInterface
    {
    public:
        AZ_RTTI(GeoJSONSpawnerROS2Module, GeoJSONSpawnerROS2ModuleTypeId, GeoJSONSpawnerROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoJSONSpawnerROS2Module, AZ::SystemAllocator);
    };
} // namespace GeoJSONSpawnerROS2

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), GeoJSONSpawnerROS2::GeoJSONSpawnerROS2Module)
#else
AZ_DECLARE_MODULE_CLASS(Gem_GeoJSONSpawnerROS2, GeoJSONSpawnerROS2::GeoJSONSpawnerROS2Module)
#endif
