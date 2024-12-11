/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>

namespace GeoJSONSpawnerROS2
{
    inline constexpr const char* stringMessageType = "std_msgs::msg::String";
    inline constexpr const char* emptyMessageType = "std_msgs::msg::Empty";
    inline constexpr const char* int32MultiArrayMessageType = "std_msgs::msg::Int32MultiArray";

    GeoJSONSpawnerROS2Configuration::GeoJSONSpawnerROS2Configuration()
    {
        m_spawnWithRawStringTopicConfiguration.m_topic = "geojson/spawn_with_raw_string";
        m_spawnWithRawStringTopicConfiguration.m_type = stringMessageType;

        m_spawnWithAssetPathTopicConfiguration.m_topic = "geojson/spawn_with_asset_path";
        m_spawnWithAssetPathTopicConfiguration.m_type = stringMessageType;

        m_modifyTopicConfiguration.m_topic = "geojson/modify";
        m_modifyTopicConfiguration.m_type = stringMessageType;

        m_deleteByIdTopicConfiguration.m_topic = "geojson/delete_by_id";
        m_deleteByIdTopicConfiguration.m_type = int32MultiArrayMessageType;

        m_deleteAllTopicConfiguration.m_topic = "geojson/delete_all";
        m_deleteAllTopicConfiguration.m_type = emptyMessageType;
    }

    void GeoJSONSpawnerROS2Configuration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoJSONSpawnerROS2Configuration>()
                ->Version(0)
                ->Field("GetIdsServiceTopicName", &GeoJSONSpawnerROS2Configuration::m_getIdsServiceTopicName)
                ->Field("SpawnWithRawStringTopicConfiguration", &GeoJSONSpawnerROS2Configuration::m_spawnWithRawStringTopicConfiguration)
                ->Field("SpawnWithAssetPathTopicConfiguration", &GeoJSONSpawnerROS2Configuration::m_spawnWithAssetPathTopicConfiguration)
                ->Field("ModifyTopicConfiguration", &GeoJSONSpawnerROS2Configuration::m_modifyTopicConfiguration)
                ->Field("DeleteAllTopicConfiguration", &GeoJSONSpawnerROS2Configuration::m_deleteAllTopicConfiguration)
                ->Field("DeleteByIdTopicConfiguration", &GeoJSONSpawnerROS2Configuration::m_deleteByIdTopicConfiguration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<GeoJSONSpawnerROS2Configuration>("GeoJSONSpawnerROS2Configuration", "GeoJSONSpawnerROS2Configuration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSONSpawnerROS2Configuration")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2Configuration::m_getIdsServiceTopicName,
                        "Get Ids Service Name",
                        "The name of the get ids service.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2Configuration::m_spawnWithRawStringTopicConfiguration,
                        "Spawn with raw string Topic Configuration",
                        "The spawn with raw string topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2Configuration::m_spawnWithAssetPathTopicConfiguration,
                        "Spawn with asset path Topic Configuration",
                        "The spawn with asset path topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2Configuration::m_modifyTopicConfiguration,
                        "Modify Topic Configuration",
                        "The modify topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2Configuration::m_deleteAllTopicConfiguration,
                        "Delete All Topic Configuration",
                        "The delete all enitities topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2Configuration::m_deleteByIdTopicConfiguration,
                        "Delete By Id Topic Configuration",
                        "The delete group of entities by id topic configuration.");
            }
        }
    }

    GeoJSONSpawnerROS2::GeoJSONSpawnerROS2(const GeoJSONSpawnerROS2Configuration& configuration)
        : m_configuration(configuration)
    {
    }

    void GeoJSONSpawnerROS2::Reflect(AZ::ReflectContext* context)
    {
        GeoJSONSpawnerROS2Configuration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoJSONSpawnerROS2, AZ::Component>()->Version(0)->Field("Configuration", &GeoJSONSpawnerROS2::m_configuration);
        }
    }

    void GeoJSONSpawnerROS2::Activate()
    {
        auto ros2Frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Unable to get pointer to ROS 2 frame component.");
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "Unable to get pointer to ROS 2 node.");

        if (!m_spawnWithRawStringSubscription)
        {
            const auto fullTopic = ROS2::ROS2Names::GetNamespacedName(
                ros2Frame->GetNamespace(), m_configuration.m_spawnWithRawStringTopicConfiguration.m_topic);

            m_spawnWithRawStringSubscription = ros2Node->create_subscription<StringMsg>(
                fullTopic.data(),
                m_configuration.m_spawnWithRawStringTopicConfiguration.GetQoS(),
                [this](const StringMsg& msg)
                {
                    ProcessSpawnWithRawStringMessage(msg);
                });
        }

        if (!m_spawnWithAssetPathSubscription)
        {
            const auto fullTopic = ROS2::ROS2Names::GetNamespacedName(
                ros2Frame->GetNamespace(), m_configuration.m_spawnWithAssetPathTopicConfiguration.m_topic);

            m_spawnWithAssetPathSubscription = ros2Node->create_subscription<StringMsg>(
                fullTopic.data(),
                m_configuration.m_spawnWithAssetPathTopicConfiguration.GetQoS(),
                [this](const StringMsg& msg)
                {
                    ProcessSpawnWithAssetPathMessage(msg);
                });
        }

        if (!m_modifySubscription)
        {
            const auto fullTopic =
                ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_configuration.m_modifyTopicConfiguration.m_topic);

            m_modifySubscription = ros2Node->create_subscription<StringMsg>(
                fullTopic.data(),
                m_configuration.m_modifyTopicConfiguration.GetQoS(),
                [this](const StringMsg& msg)
                {
                    ProcessModifyMessage(msg);
                });
        }

        if (!m_deleteByIdSubscription)
        {
            const auto fullTopic =
                ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_configuration.m_deleteByIdTopicConfiguration.m_topic);

            m_deleteByIdSubscription = ros2Node->create_subscription<Int32MultiArrayMsg>(
                fullTopic.data(),
                m_configuration.m_deleteByIdTopicConfiguration.GetQoS(),
                [this](const Int32MultiArrayMsg& msg)
                {
                    ProcessDeleteByIdMessage(msg);
                });
        }

        if (!m_deleteAllSubscription)
        {
            const auto fullTopic =
                ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_configuration.m_deleteAllTopicConfiguration.m_topic);

            m_deleteAllSubscription = ros2Node->create_subscription<EmptyMsg>(
                fullTopic.data(),
                m_configuration.m_deleteAllTopicConfiguration.GetQoS(),
                [this]([[maybe_unused]] const EmptyMsg& msg)
                {
                    ProcessDeleteAllMessage();
                });
        }

        if (!m_getIdsService)
        {
            const auto fullTopic = ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_configuration.m_getIdsServiceTopicName);

            m_getIdsService = ros2Node->create_service<std_srvs::srv::Trigger>(
                m_configuration.m_getIdsServiceTopicName.data(),
                [this](const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
                {
                    const auto result = GetIds();
                    response->message = result.IsSuccess() ? result.GetValue().c_str() : result.GetError().c_str();
                    response->success = result.IsSuccess() ? true : false;
                });
        }
    }

    void GeoJSONSpawnerROS2::Deactivate()
    {
        if (m_spawnWithRawStringSubscription)
        {
            m_spawnWithRawStringSubscription.reset();
        }

        if (m_spawnWithAssetPathSubscription)
        {
            m_spawnWithAssetPathSubscription.reset();
        }

        if (m_modifySubscription)
        {
            m_modifySubscription.reset();
        }

        if (m_deleteAllSubscription)
        {
            m_deleteAllSubscription.reset();
        }

        if (m_deleteByIdSubscription)
        {
            m_deleteByIdSubscription.reset();
        }

        if (m_getIdsService)
        {
            m_getIdsService.reset();
        }
    }

    void GeoJSONSpawnerROS2::ProcessSpawnWithRawStringMessage(const StringMsg& message)
    {
        GeoJSONSpawner::Result result;
        GeoJSONSpawner::GeoJSONSpawnerRequestBus::BroadcastResult(
            result, &GeoJSONSpawner::GeoJSONSpawnerRequestBus::Events::SpawnWithRawString, message.data.c_str());
        AZ_Error("GeoJSONSpawnerROS2", result.IsSuccess(), result.GetError().c_str());
    }

    void GeoJSONSpawnerROS2::ProcessSpawnWithAssetPathMessage(const StringMsg& message)
    {
        GeoJSONSpawner::Result result;
        GeoJSONSpawner::GeoJSONSpawnerRequestBus::BroadcastResult(
            result, &GeoJSONSpawner::GeoJSONSpawnerRequestBus::Events::SpawnWithAssetPath, message.data.c_str());
        AZ_Error("GeoJSONSpawnerROS2", result.IsSuccess(), result.GetError().c_str());
    }

    void GeoJSONSpawnerROS2::ProcessModifyMessage(const StringMsg& message)
    {
        GeoJSONSpawner::Result result;
        GeoJSONSpawner::GeoJSONSpawnerRequestBus::BroadcastResult(
            result, &GeoJSONSpawner::GeoJSONSpawnerRequestBus::Events::Modify, message.data.c_str());
        AZ_Error("GeoJSONSpawnerROS2", result.IsSuccess(), result.GetError().c_str());
    }

    void GeoJSONSpawnerROS2::ProcessDeleteAllMessage()
    {
        GeoJSONSpawner::Result result;
        GeoJSONSpawner::GeoJSONSpawnerRequestBus::BroadcastResult(result, &GeoJSONSpawner::GeoJSONSpawnerRequestBus::Events::DeleteAll);
        AZ_Error("GeoJSONSpawnerROS2", result.IsSuccess(), result.GetError().c_str());
    }

    void GeoJSONSpawnerROS2::ProcessDeleteByIdMessage(const Int32MultiArrayMsg& message)
    {
        GeoJSONSpawner::Result result;
        AZStd::unordered_set<int> idsToDelete;
        AZStd::copy(message.data.begin(), message.data.end(), AZStd::inserter(idsToDelete, idsToDelete.end()));
        GeoJSONSpawner::GeoJSONSpawnerRequestBus::BroadcastResult(
            result, &GeoJSONSpawner::GeoJSONSpawnerRequestBus::Events::DeleteById, idsToDelete);
        AZ_Error("GeoJSONSpawnerROS2", result.IsSuccess(), result.GetError().c_str());
    }

    GeoJSONSpawner::GetIdsResult GeoJSONSpawnerROS2::GetIds()
    {
        GeoJSONSpawner::GetIdsResult result;
        GeoJSONSpawner::GeoJSONSpawnerRequestBus::BroadcastResult(result, &GeoJSONSpawner::GeoJSONSpawnerRequestBus::Events::GetIds);
        return result;
    }

} // namespace GeoJSONSpawnerROS2