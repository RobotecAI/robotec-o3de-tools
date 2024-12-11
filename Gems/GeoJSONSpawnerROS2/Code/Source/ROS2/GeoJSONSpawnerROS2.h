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

#include <GeoJSONSpawnerROS2/GeoJSONSpawnerROS2TypeIds.h>

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <GeoJSONSpawner/GeoJSONSpawnerBus.h>
#include <ROS2/RobotControl/ControlSubscriptionHandler.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace GeoJSONSpawnerROS2
{
    struct GeoJSONSpawnerROS2Configuration
    {
        AZ_TYPE_INFO(GeoJSONSpawnerROS2Configuration, GeoJSONSpawnerROS2ConfigurationTypeId);
        GeoJSONSpawnerROS2Configuration();
        ~GeoJSONSpawnerROS2Configuration() = default;
        static void Reflect(AZ::ReflectContext* context);

        ROS2::TopicConfiguration m_spawnWithRawStringTopicConfiguration;
        ROS2::TopicConfiguration m_spawnWithAssetPathTopicConfiguration;
        ROS2::TopicConfiguration m_modifyTopicConfiguration;
        ROS2::TopicConfiguration m_deleteAllTopicConfiguration;
        ROS2::TopicConfiguration m_deleteByIdTopicConfiguration;
        AZStd::string m_getIdsServiceTopicName{ "geojson/get_spawned_groups_ids" };
    };

    class GeoJSONSpawnerROS2 : public AZ::Component
    {
    public:
        AZ_COMPONENT(GeoJSONSpawnerROS2, GeoJSONSpawnerROS2TypeId);

        GeoJSONSpawnerROS2() = default;
        GeoJSONSpawnerROS2(const GeoJSONSpawnerROS2Configuration& configuration);
        ~GeoJSONSpawnerROS2() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:
        using StringMsg = std_msgs::msg::String;
        using EmptyMsg = std_msgs::msg::Empty;
        using Int32MultiArrayMsg = std_msgs::msg::Int32MultiArray;

        void ProcessSpawnWithRawStringMessage(const StringMsg& message);
        void ProcessSpawnWithAssetPathMessage(const StringMsg& message);
        void ProcessModifyMessage(const StringMsg& message);
        void ProcessDeleteAllMessage();
        void ProcessDeleteByIdMessage(const Int32MultiArrayMsg& message);
        GeoJSONSpawner::GetIdsResult GetIds();

        GeoJSONSpawnerROS2Configuration m_configuration;

        rclcpp::Subscription<StringMsg>::SharedPtr m_spawnWithRawStringSubscription;
        rclcpp::Subscription<StringMsg>::SharedPtr m_spawnWithAssetPathSubscription;
        rclcpp::Subscription<StringMsg>::SharedPtr m_modifySubscription;
        rclcpp::Subscription<Int32MultiArrayMsg>::SharedPtr m_deleteByIdSubscription;
        rclcpp::Subscription<EmptyMsg>::SharedPtr m_deleteAllSubscription;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_getIdsService;
    };

} // namespace GeoJSONSpawnerROS2
