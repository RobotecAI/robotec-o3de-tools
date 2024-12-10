/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "GeoJSONSpawnerROS2.h"
#include <GeoJSONSpawnerROS2/GeoJSONSpawnerROS2TypeIds.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace GeoJSONSpawnerROS2
{
    class GeoJSONSpawnerROS2EditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(GeoJSONSpawnerROS2EditorComponent, ::GeoJSONSpawnerROS2::GeoJSONSpawnerROS2EditorComponentTypeId);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        GeoJSONSpawnerROS2EditorComponent() = default;
        ~GeoJSONSpawnerROS2EditorComponent() = default;

        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        GeoJSONSpawnerROS2Configuration m_configuration;
    };

} // namespace GeoJSONSpawnerROS2
