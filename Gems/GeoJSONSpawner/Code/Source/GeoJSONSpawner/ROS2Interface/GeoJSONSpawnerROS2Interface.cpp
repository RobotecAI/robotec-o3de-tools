/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2Interface.h"

#include "AzCore/Serialization/EditContext.h"
#include "GeoJSONSpawner/GeoJSONSpawnerBus.h"
#include "SubscriptionHandler.h"

namespace GeoJSONSpawner::ROS2Interface
{
    void GeoJSONSpawnerROS2InterfaceConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoJSONSpawnerROS2InterfaceConfiguration>()
                ->Version(0)
                ->Field("SpawnTopicConfiguration", &GeoJSONSpawnerROS2InterfaceConfiguration::m_spawnTopicConfiguration)
                ->Field("ModifyTopicConfiguration", &GeoJSONSpawnerROS2InterfaceConfiguration::m_modifyTopicConfiguration)
                ->Field("DeleteAllTopicConfiguration", &GeoJSONSpawnerROS2InterfaceConfiguration::m_deleteAllTopicConfiguration)
                ->Field("DeleteByIdTopicConfiguration", &GeoJSONSpawnerROS2InterfaceConfiguration::m_deleteByIdTopicConfiguration)
                ->Field("GetIdsServiceTopicName", &GeoJSONSpawnerROS2InterfaceConfiguration::m_getIdsServiceTopicName);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<GeoJSONSpawnerROS2InterfaceConfiguration>(
                        "GeoJSONSpawnerROS2InterfaceConfiguration", "GeoJSONSpawnerROS2InterfaceConfiguration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GeoJSONSpawnerROS2InterfaceConfiguration")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Spawners")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2InterfaceConfiguration::m_spawnTopicConfiguration,
                        "SpawnTopicConfiguration",
                        "The spawn topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2InterfaceConfiguration::m_modifyTopicConfiguration,
                        "ModifyTopicConfiguration",
                        "The modify topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2InterfaceConfiguration::m_deleteAllTopicConfiguration,
                        "DeleteAllTopicConfiguration",
                        "The delete all enitities topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2InterfaceConfiguration::m_deleteByIdTopicConfiguration,
                        "DeleteByIdTopicConfiguration",
                        "The delete group of entities by id topic configuration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoJSONSpawnerROS2InterfaceConfiguration::m_getIdsServiceTopicName,
                        "GetIdsServiceTopicName",
                        "The name of the service topic.");
            }
        }
    }

    GeoJSONSpawnerROS2Interface::GeoJSONSpawnerROS2Interface(const GeoJSONSpawnerROS2InterfaceConfiguration& configuration)
        : m_configuration(configuration)
    {
    }

    void GeoJSONSpawnerROS2Interface::Reflect(AZ::ReflectContext* context)
    {
        GeoJSONSpawnerROS2InterfaceConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoJSONSpawnerROS2Interface, AZ::Component>()->Version(0)->Field(
                "Configuration", &GeoJSONSpawnerROS2Interface::m_configuration);
        }
    }

    void GeoJSONSpawnerROS2Interface::Activate()
    {
        m_spawnSubscriptionHandler = AZStd::make_unique<SubscriptionHandler<StringMsg>>(
            [this](const StringMsg& message)
            {
                ProcessSpawnMessage(message);
            });
        m_spawnSubscriptionHandler->Activate(GetEntity(), m_configuration.m_spawnTopicConfiguration);

        m_modifySubscriptionHandler = AZStd::make_unique<SubscriptionHandler<StringMsg>>(
            [this](const StringMsg& message)
            {
                ProcessModifyMessage(message);
            });
        m_modifySubscriptionHandler->Activate(GetEntity(), m_configuration.m_modifyTopicConfiguration);

        m_deleteAllSubscriptionHandler = AZStd::make_unique<SubscriptionHandler<EmptyMsg>>(
            [this](const EmptyMsg& message)
            {
                ProcessDeleteAllMessage();
            });
        m_deleteAllSubscriptionHandler->Activate(GetEntity(), m_configuration.m_deleteAllTopicConfiguration);

        m_deleteByIdSubscriptionHandler = AZStd::make_unique<SubscriptionHandler<StringMsg>>(
            [this](const StringMsg& message)
            {
                ProcessDeleteByIdMessage(message);
            });
        m_deleteByIdSubscriptionHandler->Activate(GetEntity(), m_configuration.m_deleteByIdTopicConfiguration);

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
            const auto fullTopic = ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_configuration.m_getIdsServiceTopicName);

            m_getIdsService = ros2Node->create_service<std_srvs::srv::Trigger>(
                m_configuration.m_getIdsServiceTopicName.c_str(),
                [this](const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
                {
                    response->message = GetIds().c_str();
                    response->success = true;
                });
        }
    }

    void GeoJSONSpawnerROS2Interface::Deactivate()
    {
        if (m_spawnSubscriptionHandler)
        {
            m_spawnSubscriptionHandler->Deactivate();
            m_spawnSubscriptionHandler.reset();
        }

        if (m_modifySubscriptionHandler)
        {
            m_modifySubscriptionHandler->Deactivate();
            m_modifySubscriptionHandler.reset();
        }

        if (m_deleteAllSubscriptionHandler)
        {
            m_deleteAllSubscriptionHandler->Deactivate();
            m_deleteAllSubscriptionHandler.reset();
        }

        if (m_deleteByIdSubscriptionHandler)
        {
            m_deleteByIdSubscriptionHandler->Deactivate();
            m_deleteByIdSubscriptionHandler.reset();
        }

        if (m_getIdsService)
        {
            m_getIdsService.reset();
        }
    }

    void GeoJSONSpawnerROS2Interface::ProcessSpawnMessage(const StringMsg& message)
    {
        GeoJSONSpawnerRequestBus::Broadcast(&GeoJSONSpawnerRequestBus::Events::Spawn, message.data.c_str());
    }

    void GeoJSONSpawnerROS2Interface::ProcessModifyMessage(const StringMsg& message)
    {
        GeoJSONSpawnerRequestBus::Broadcast(&GeoJSONSpawnerRequestBus::Events::Modify, message.data.c_str());
    }

    void GeoJSONSpawnerROS2Interface::ProcessDeleteAllMessage()
    {
        GeoJSONSpawnerRequestBus::Broadcast(&GeoJSONSpawnerRequestBus::Events::DeleteAll);
    }

    void GeoJSONSpawnerROS2Interface::ProcessDeleteByIdMessage(const StringMsg& message)
    {
        GeoJSONSpawnerRequestBus::Broadcast(&GeoJSONSpawnerRequestBus::Events::DeleteById, message.data.c_str());
    }

    AZStd::string GeoJSONSpawnerROS2Interface::GetIds()
    {
        AZStd::string result;
        GeoJSONSpawnerRequestBus::BroadcastResult(result, &GeoJSONSpawnerRequestBus::Events::GetIds);
        return result;
    }


} // namespace GeoJSONSpawner::ROS2Interface