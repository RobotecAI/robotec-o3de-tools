/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/RobotecGeoJSONSpawnerROS2SystemComponent.h>

namespace RobotecGeoJSONSpawnerROS2
{
    /// System component for RobotecGeoJSONSpawnerROS2 editor
    class RobotecGeoJSONSpawnerROS2EditorSystemComponent
        : public RobotecGeoJSONSpawnerROS2SystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = RobotecGeoJSONSpawnerROS2SystemComponent;

    public:
        AZ_COMPONENT_DECL(RobotecGeoJSONSpawnerROS2EditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        RobotecGeoJSONSpawnerROS2EditorSystemComponent();
        ~RobotecGeoJSONSpawnerROS2EditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace RobotecGeoJSONSpawnerROS2
