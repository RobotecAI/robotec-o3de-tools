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

#include <RobotecGeoJSONSpawnerROS2/RobotecGeoJSONSpawnerROS2TypeIds.h>

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2/RobotControl/ControlSubscriptionHandler.h>
#include <RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerBus.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace RobotecGeoJSONSpawnerROS2
{
    struct RobotecGeoJSONSpawnerROS2Configuration
    {
        AZ_TYPE_INFO(RobotecGeoJSONSpawnerROS2Configuration, RobotecGeoJSONSpawnerROS2ConfigurationTypeId);
        RobotecGeoJSONSpawnerROS2Configuration();
        ~RobotecGeoJSONSpawnerROS2Configuration() = default;
        static void Reflect(AZ::ReflectContext* context);

        ROS2::TopicConfiguration m_spawnWithRawStringTopicConfiguration;
        ROS2::TopicConfiguration m_spawnWithAssetPathTopicConfiguration;
        ROS2::TopicConfiguration m_modifyTopicConfiguration;
        ROS2::TopicConfiguration m_deleteAllTopicConfiguration;
        ROS2::TopicConfiguration m_deleteByIdTopicConfiguration;
        AZStd::string m_getIdsServiceTopicName{ "geojson/get_spawned_groups_ids" };
    };

    class RobotecGeoJSONSpawnerROS2 : public AZ::Component
    {
    public:
        AZ_COMPONENT(RobotecGeoJSONSpawnerROS2, RobotecGeoJSONSpawnerROS2TypeId);

        RobotecGeoJSONSpawnerROS2() = default;
        RobotecGeoJSONSpawnerROS2(const RobotecGeoJSONSpawnerROS2Configuration& configuration);
        ~RobotecGeoJSONSpawnerROS2() = default;

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
        RobotecGeoJSONSpawner::GetIdsResult GetIds();

        RobotecGeoJSONSpawnerROS2Configuration m_configuration;

        rclcpp::Subscription<StringMsg>::SharedPtr m_spawnWithRawStringSubscription;
        rclcpp::Subscription<StringMsg>::SharedPtr m_spawnWithAssetPathSubscription;
        rclcpp::Subscription<StringMsg>::SharedPtr m_modifySubscription;
        rclcpp::Subscription<Int32MultiArrayMsg>::SharedPtr m_deleteByIdSubscription;
        rclcpp::Subscription<EmptyMsg>::SharedPtr m_deleteAllSubscription;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_getIdsService;
    };

} // namespace RobotecGeoJSONSpawnerROS2
