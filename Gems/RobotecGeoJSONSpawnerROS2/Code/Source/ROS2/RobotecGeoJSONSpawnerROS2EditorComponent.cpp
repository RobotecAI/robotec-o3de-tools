/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotecGeoJSONSpawnerROS2EditorComponent.h"

#include <AzCore/Serialization/EditContext.h>

namespace RobotecGeoJSONSpawnerROS2
{
    void RobotecGeoJSONSpawnerROS2EditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RobotecGeoJSONSpawnerROS2EditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(0)
                ->Field("Configuration", &RobotecGeoJSONSpawnerROS2EditorComponent::m_configuration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<RobotecGeoJSONSpawnerROS2EditorComponent>(
                        "RobotecGeoJSONSpawnerROS2EditorComponent", "RobotecGeoJSONSpawnerROS2EditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "RobotecGeoJSONSpawnerROS2EditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RobotecGeoJSONSpawnerROS2EditorComponent::m_configuration,
                        "Configuration",
                        "Configuration of the spawner's ROS2 interface.");
            }
        }
    }

    void RobotecGeoJSONSpawnerROS2EditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerROS2"));
    }

    void RobotecGeoJSONSpawnerROS2EditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerROS2"));
    }

    void RobotecGeoJSONSpawnerROS2EditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void RobotecGeoJSONSpawnerROS2EditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<RobotecGeoJSONSpawnerROS2>(m_configuration);
    }

} // namespace RobotecGeoJSONSpawnerROS2
