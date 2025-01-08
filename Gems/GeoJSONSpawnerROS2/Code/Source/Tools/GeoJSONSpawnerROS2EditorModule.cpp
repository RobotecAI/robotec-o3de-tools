/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2EditorSystemComponent.h"
#include <GeoJSONSpawnerROS2/GeoJSONSpawnerROS2TypeIds.h>
#include <GeoJSONSpawnerROS2ModuleInterface.h>
#include <ROS2/GeoJSONSpawnerROS2EditorComponent.h>

namespace GeoJSONSpawnerROS2
{
    class GeoJSONSpawnerROS2EditorModule : public GeoJSONSpawnerROS2ModuleInterface
    {
    public:
        AZ_RTTI(GeoJSONSpawnerROS2EditorModule, GeoJSONSpawnerROS2EditorModuleTypeId, GeoJSONSpawnerROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoJSONSpawnerROS2EditorModule, AZ::SystemAllocator);

        GeoJSONSpawnerROS2EditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                { GeoJSONSpawnerROS2EditorSystemComponent::CreateDescriptor(), GeoJSONSpawnerROS2EditorComponent::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<GeoJSONSpawnerROS2EditorSystemComponent>(),
            };
        }
    };
} // namespace GeoJSONSpawnerROS2

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), GeoJSONSpawnerROS2::GeoJSONSpawnerROS2EditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_GeoJSONSpawnerROS2_Editor, GeoJSONSpawnerROS2::GeoJSONSpawnerROS2EditorModule)
#endif
