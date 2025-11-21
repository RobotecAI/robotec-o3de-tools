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

#include <Clients/RobotecGeoJSONSpawnerSystemComponent.h>

namespace RobotecGeoJSONSpawner
{
    /// System component for RobotecGeoJSONSpawner editor
    class RobotecGeoJSONSpawnerEditorSystemComponent
        : public RobotecGeoJSONSpawnerSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = RobotecGeoJSONSpawnerSystemComponent;

    public:
        AZ_COMPONENT_DECL(RobotecGeoJSONSpawnerEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        RobotecGeoJSONSpawnerEditorSystemComponent() = default;
        ~RobotecGeoJSONSpawnerEditorSystemComponent() = default;

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace RobotecGeoJSONSpawner
