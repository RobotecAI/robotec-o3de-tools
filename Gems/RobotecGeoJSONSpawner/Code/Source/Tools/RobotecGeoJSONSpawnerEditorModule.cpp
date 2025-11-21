/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
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
