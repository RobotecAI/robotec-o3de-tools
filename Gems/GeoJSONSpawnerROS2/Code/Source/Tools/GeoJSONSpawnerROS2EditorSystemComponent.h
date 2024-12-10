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

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/GeoJSONSpawnerROS2SystemComponent.h>

namespace GeoJSONSpawnerROS2
{
    /// System component for GeoJSONSpawnerROS2 editor
    class GeoJSONSpawnerROS2EditorSystemComponent
        : public GeoJSONSpawnerROS2SystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = GeoJSONSpawnerROS2SystemComponent;

    public:
        AZ_COMPONENT_DECL(GeoJSONSpawnerROS2EditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        GeoJSONSpawnerROS2EditorSystemComponent();
        ~GeoJSONSpawnerROS2EditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace GeoJSONSpawnerROS2
