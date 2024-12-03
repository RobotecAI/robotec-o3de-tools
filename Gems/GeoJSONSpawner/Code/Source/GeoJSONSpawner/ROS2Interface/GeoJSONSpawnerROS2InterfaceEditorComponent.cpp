/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2InterfaceEditorComponent.h"

#include "AzCore/Serialization/EditContext.h"

namespace GeoJSONSpawner::ROS2Interface
{
    inline constexpr const char* stringMessageType = "std_msgs::msg::String";
    inline constexpr const char* emptyMessageType = "std_msgs::msg::Empty";

    GeoJSONSpawnerROS2InterfaceEditorComponent::GeoJSONSpawnerROS2InterfaceEditorComponent()
    {
        m_configuration.m_spawnTopicConfiguration.m_topic = "geojson/spawn";
        m_configuration.m_spawnTopicConfiguration.m_type = stringMessageType;

        m_configuration.m_modifyTopicConfiguration.m_topic = "geojson/modify";
        m_configuration.m_modifyTopicConfiguration.m_type = stringMessageType;

        m_configuration.m_deleteByIdTopicConfiguration.m_topic = "geojson/delete_by_id";
        m_configuration.m_deleteByIdTopicConfiguration.m_type = stringMessageType;

        m_configuration.m_deleteAllTopicConfiguration.m_topic = "geojson/delete_all";
        m_configuration.m_deleteAllTopicConfiguration.m_type = emptyMessageType;
    }

    void GeoJSONSpawnerROS2InterfaceEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoJSONSpawnerROS2InterfaceEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(0)
                ->Field("Configuration", &GeoJSONSpawnerROS2InterfaceEditorComponent::m_configuration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<GeoJSONSpawnerROS2InterfaceEditorComponent>(
                        "GeoJSONSpawnerROS2InterfaceEditorComponent", "GeoJSONSpawnerROS2InterfaceEditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSONSpawnerROS2InterfaceEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2InterfaceEditorComponent::m_configuration,
                        "Configuration",
                        "Configuration of the spawner's ROS2 interface.");
            }
        }
    }

    void GeoJSONSpawnerROS2InterfaceEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GeoJSONSpawner"));
    }

    void GeoJSONSpawnerROS2InterfaceEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GeoJSONSpawner"));
    }

    void GeoJSONSpawnerROS2InterfaceEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void GeoJSONSpawnerROS2InterfaceEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<GeoJSONSpawnerROS2Interface>(m_configuration);
    }

} // namespace GeoJSONSpawner::ROS2Interface