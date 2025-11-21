/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "RobotecGeoJSONSpawnerROS2EditorSystemComponent.h"
#include <ROS2/RobotecGeoJSONSpawnerROS2EditorComponent.h>
#include <RobotecGeoJSONSpawnerROS2/RobotecGeoJSONSpawnerROS2TypeIds.h>
#include <RobotecGeoJSONSpawnerROS2ModuleInterface.h>

namespace RobotecGeoJSONSpawnerROS2
{
    class RobotecGeoJSONSpawnerROS2EditorModule : public RobotecGeoJSONSpawnerROS2ModuleInterface
    {
    public:
        AZ_RTTI(
            RobotecGeoJSONSpawnerROS2EditorModule, RobotecGeoJSONSpawnerROS2EditorModuleTypeId, RobotecGeoJSONSpawnerROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecGeoJSONSpawnerROS2EditorModule, AZ::SystemAllocator);

        RobotecGeoJSONSpawnerROS2EditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                { RobotecGeoJSONSpawnerROS2EditorSystemComponent::CreateDescriptor(),
                  RobotecGeoJSONSpawnerROS2EditorComponent::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RobotecGeoJSONSpawnerROS2EditorSystemComponent>(),
            };
        }
    };
} // namespace RobotecGeoJSONSpawnerROS2

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), RobotecGeoJSONSpawnerROS2::RobotecGeoJSONSpawnerROS2EditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_RobotecGeoJSONSpawnerROS2_Editor, RobotecGeoJSONSpawnerROS2::RobotecGeoJSONSpawnerROS2EditorModule)
#endif
