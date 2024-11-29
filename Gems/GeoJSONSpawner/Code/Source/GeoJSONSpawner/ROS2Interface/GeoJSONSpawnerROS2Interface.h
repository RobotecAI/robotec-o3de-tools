/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "GeoJSONSpawner/GeoJSONSpawnerTypeIds.h"

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2/RobotControl/ControlSubscriptionHandler.h>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace GeoJSONSpawner::ROS2Interface
{
    struct GeoJSONSpawnerROS2InterfaceConfiguration
    {
        AZ_TYPE_INFO(GeoJSONSpawnerROS2InterfaceConfiguration, GeoJSONSpawnerROS2InterfaceConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        ROS2::TopicConfiguration m_spawnTopicConfiguration;
        ROS2::TopicConfiguration m_modifyTopicConfiguration;
        ROS2::TopicConfiguration m_deleteAllTopicConfiguration;
        ROS2::TopicConfiguration m_deleteByIdTopicConfiguration;
        AZStd::string m_getIdsServiceTopicName{ "geojson/get_spawned_groups_ids" };
    };

    class GeoJSONSpawnerROS2Interface : public AZ::Component
    {
    public:
        AZ_COMPONENT(GeoJSONSpawnerROS2Interface, GeoJSONSpawnerROS2InterfaceTypeId);

        GeoJSONSpawnerROS2Interface() = default;
        GeoJSONSpawnerROS2Interface(const GeoJSONSpawnerROS2InterfaceConfiguration& configuration);
        ~GeoJSONSpawnerROS2Interface() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:
        using StringMsg = std_msgs::msg::String;
        using EmptyMsg = std_msgs::msg::Empty;

        void ProcessSpawnMessage(const StringMsg& message);
        void ProcessModifyMessage(const StringMsg& message);
        void ProcessDeleteAllMessage();
        void ProcessDeleteByIdMessage(const StringMsg& message);
        AZStd::string GetIds();

        GeoJSONSpawnerROS2InterfaceConfiguration m_configuration;

        AZStd::unique_ptr<ROS2::IControlSubscriptionHandler> m_spawnSubscriptionHandler;
        AZStd::unique_ptr<ROS2::IControlSubscriptionHandler> m_modifySubscriptionHandler;
        AZStd::unique_ptr<ROS2::IControlSubscriptionHandler> m_deleteAllSubscriptionHandler;
        AZStd::unique_ptr<ROS2::IControlSubscriptionHandler> m_deleteByIdSubscriptionHandler;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_getIdsService;
    };

} // namespace GeoJSONSpawner::ROS2Interface
