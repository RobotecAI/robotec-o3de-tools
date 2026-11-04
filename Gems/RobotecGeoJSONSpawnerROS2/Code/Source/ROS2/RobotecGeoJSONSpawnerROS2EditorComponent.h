/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "RobotecGeoJSONSpawnerROS2.h"
#include <RobotecGeoJSONSpawnerROS2/RobotecGeoJSONSpawnerROS2TypeIds.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace RobotecGeoJSONSpawnerROS2
{
    class RobotecGeoJSONSpawnerROS2EditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(
            RobotecGeoJSONSpawnerROS2EditorComponent, ::RobotecGeoJSONSpawnerROS2::RobotecGeoJSONSpawnerROS2EditorComponentTypeId);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        RobotecGeoJSONSpawnerROS2EditorComponent() = default;
        ~RobotecGeoJSONSpawnerROS2EditorComponent() = default;

        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        RobotecGeoJSONSpawnerROS2Configuration m_configuration;
    };

} // namespace RobotecGeoJSONSpawnerROS2
