/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerEditorComponent.h"
#include "RobotecGeoJSONSpawnerEditorSystemComponent.h"
#include <RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerTypeIds.h>
#include <RobotecGeoJSONSpawnerModuleInterface.h>

namespace RobotecGeoJSONSpawner
{
    class RobotecGeoJSONSpawnerEditorModule : public RobotecGeoJSONSpawnerModuleInterface
    {
    public:
        AZ_RTTI(RobotecGeoJSONSpawnerEditorModule, RobotecGeoJSONSpawnerEditorModuleTypeId, RobotecGeoJSONSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecGeoJSONSpawnerEditorModule, AZ::SystemAllocator);

        RobotecGeoJSONSpawnerEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                { RobotecGeoJSONSpawnerEditorSystemComponent::CreateDescriptor(),
                  RobotecGeoJSONSpawnerEditorComponent::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RobotecGeoJSONSpawnerEditorSystemComponent>(),
            };
        }
    };
} // namespace RobotecGeoJSONSpawner

AZ_DECLARE_MODULE_CLASS(Gem_RobotecGeoJSONSpawner, RobotecGeoJSONSpawner::RobotecGeoJSONSpawnerEditorModule)
