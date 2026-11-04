/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <AzCore/Memory/Memory_fwd.h>
#include <AzCore/Module/Module.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace RobotecGeoJSONSpawner
{
    class RobotecGeoJSONSpawnerModuleInterface : public AZ::Module
    {
    public:
        AZ_TYPE_INFO_WITH_NAME_DECL(RobotecGeoJSONSpawnerModuleInterface)
        AZ_RTTI_NO_TYPE_INFO_DECL()
        AZ_CLASS_ALLOCATOR_DECL

        RobotecGeoJSONSpawnerModuleInterface();

        AZ::ComponentTypeList GetRequiredSystemComponents() const override;
    };
} // namespace RobotecGeoJSONSpawner
