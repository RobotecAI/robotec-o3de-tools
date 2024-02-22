
#include "SubscriberSystemComponent.h"
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>
#include <ROS2ScriptIntegration/ROS2ScriptSubscriberBus.h>

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

namespace ROS2ScriptIntegration
{
    AZ_COMPONENT_IMPL(SubscriberSystemComponent, "SubscriberSystemComponent", SubscriberSystemComponentTypeId);

    void SubscriberSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        SubscriberRequests::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SubscriberSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void SubscriberSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2ScriptIntegrationSubscriberService"));
    }

    void SubscriberSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2ScriptIntegrationSubscriberService"));
    }

    void SubscriberSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void SubscriberSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    SubscriberSystemComponent::SubscriberSystemComponent()
    {
        if (SubscriberInterface::Get() == nullptr)
        {
            SubscriberInterface::Register(this);
        }
    }

    SubscriberSystemComponent::~SubscriberSystemComponent()
    {
        if (SubscriberInterface::Get() == this)
        {
            SubscriberInterface::Unregister(this);
        }
    }

    void SubscriberSystemComponent::Init()
    {
    }

    void SubscriberSystemComponent::Activate()
    {
        SubscriberRequestBus::Handler::BusConnect();
    }

    void SubscriberSystemComponent::Deactivate()
    {
        AZStd::lock_guard<AZStd::shared_mutex> lock2(m_subscribersMapMutex);
        m_subscribers.clear();
        SubscriberRequestBus::Handler::BusDisconnect();
    }

    // SubscriberRequests overrides ...
    void SubscriberSystemComponent::SubscribeToStdMsgBool(const AZStd::string& topicName)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<std_msgs::msg::Bool>(
                std::string(topicName.c_str()),
                10,
                [](const std_msgs::msg::Bool::SharedPtr msg)
                {
                    SubscriberNotificationsBus::Broadcast(&SubscriberNotificationsBus::Events::OnStdMsgBool, msg->data);
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    }

    void SubscriberSystemComponent::SubscribeToSensorMsgJoy(const AZStd::string& topicName)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<sensor_msgs::msg::Joy>(
                std::string(topicName.c_str()),
                10,
                [](sensor_msgs::msg::Joy::SharedPtr msg)
                {
                    msg->buttons.resize(4);
                    msg->axes.resize(4);
                    SubscriberNotificationsBus::Broadcast(
                        &SubscriberNotificationsBus::Events::OnSensorMsgJoy,
                        msg->buttons[0],
                        msg->buttons[1],
                        msg->buttons[2],
                        msg->buttons[3],
                        msg->axes[0],
                        msg->axes[1],
                        msg->axes[2],
                        msg->axes[3]);
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    }

    void SubscriberSystemComponent::SubscribeToGeometryMsgPoseStamped(const AZStd::string& topicName)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<geometry_msgs::msg::PoseStamped>(
                std::string(topicName.c_str()),
                10,
                [](geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    SubscriberNotificationsBus::Broadcast(
                        &SubscriberNotificationsBus::Events::OnGeometryMsgPoseStamped,
                        AZStd::string(msg->header.frame_id.c_str()),
                        AZ::Transform::CreateFromQuaternionAndTranslation(
                            AZ::Quaternion(
                                msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
                            AZ::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)));
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    }

    void SubscriberSystemComponent::SubscribeToString(const AZStd::string& topicName)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<std_msgs::msg::String>(
                std::string(topicName.c_str()),
                10,
                [](std_msgs::msg::String::SharedPtr msg)
                {
                    SubscriberNotificationsBus::Broadcast(
                        &SubscriberNotificationsBus::Events::OnStdMsgString, AZStd::string(msg->data.c_str()));
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    }
    void SubscriberSystemComponent::SubscribeToFloat32(const AZStd::string& topicName)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<std_msgs::msg::Float32>(
                std::string(topicName.c_str()),
                10,
                [](std_msgs::msg::Float32::SharedPtr msg)
                {
                    SubscriberNotificationsBus::Broadcast(&SubscriberNotificationsBus::Events::OnStdMsgFloat32, msg->data);
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    }

    void SubscriberSystemComponent::SubscribeToUInt32(const AZStd::string& topicName)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<std_msgs::msg::UInt32>(
                std::string(topicName.c_str()),
                10,
                [](std_msgs::msg::UInt32::SharedPtr msg)
                {
                    SubscriberNotificationsBus::Broadcast(&SubscriberNotificationsBus::Events::OnStdMsgUInt32, msg->data);
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    };

    void SubscriberSystemComponent::SubscribeToInt32(const AZStd::string& topicName)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<std_msgs::msg::Int32>(
                std::string(topicName.c_str()),
                10,
                [](std_msgs::msg::Int32::SharedPtr msg)
                {
                    SubscriberNotificationsBus::Broadcast(&SubscriberNotificationsBus::Events::OnStdMsgInt32, msg->data);
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    };

} // namespace ROS2ScriptIntegration
