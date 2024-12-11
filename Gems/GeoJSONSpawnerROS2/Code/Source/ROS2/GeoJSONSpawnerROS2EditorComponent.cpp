/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2EditorComponent.h"

#include <AzCore/Serialization/EditContext.h>

namespace GeoJSONSpawnerROS2
{
    void GeoJSONSpawnerROS2EditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoJSONSpawnerROS2EditorComponent, AzToolsFramework::Components::EditorComponentBase>()->Version(0)->Field(
                "Configuration", &GeoJSONSpawnerROS2EditorComponent::m_configuration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<GeoJSONSpawnerROS2EditorComponent>("GeoJSONSpawnerROS2EditorComponent", "GeoJSONSpawnerROS2EditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSONSpawnerROS2EditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2EditorComponent::m_configuration,
                        "Configuration",
                        "Configuration of the spawner's ROS2 interface.");
            }
        }
    }

    void GeoJSONSpawnerROS2EditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GeoJSONSpawnerROS2"));
    }

    void GeoJSONSpawnerROS2EditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GeoJSONSpawnerROS2"));
    }

    void GeoJSONSpawnerROS2EditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void GeoJSONSpawnerROS2EditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<GeoJSONSpawnerROS2>(m_configuration);
    }

} // namespace GeoJSONSpawnerROS2
