/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawner/GeoJSONSpawnerEditorComponent.h"
#include "GeoJSONSpawnerEditorSystemComponent.h"
#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>
#include <GeoJSONSpawnerModuleInterface.h>

namespace GeoJSONSpawner
{
    class GeoJSONSpawnerEditorModule : public GeoJSONSpawnerModuleInterface
    {
    public:
        AZ_RTTI(GeoJSONSpawnerEditorModule, GeoJSONSpawnerEditorModuleTypeId, GeoJSONSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoJSONSpawnerEditorModule, AZ::SystemAllocator);

        GeoJSONSpawnerEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                { GeoJSONSpawnerEditorSystemComponent::CreateDescriptor(), GeoJSONSpawnerEditorComponent::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<GeoJSONSpawnerEditorSystemComponent>(),
            };
        }
    };
} // namespace GeoJSONSpawner

AZ_DECLARE_MODULE_CLASS(Gem_GeoJSONSpawner, GeoJSONSpawner::GeoJSONSpawnerEditorModule)
