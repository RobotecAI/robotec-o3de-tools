
#include "ROS2ScriptIntegrationSystemComponent.h"
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
    AZ_COMPONENT_IMPL(
        ROS2ScriptIntegrationSystemComponent, "ROS2ScriptIntegrationSystemComponent", ROS2ScriptIntegrationSystemComponentTypeId);

    void ROS2ScriptIntegrationSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        SubscriberNotificationHandler::Reflect(context);
        SubscriberRequests::Reflect(context);
        PublisherRequests::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ScriptIntegrationSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2ScriptIntegrationService"));
    }

    void ROS2ScriptIntegrationSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2ScriptIntegrationService"));
    }

    void ROS2ScriptIntegrationSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void ROS2ScriptIntegrationSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2ScriptIntegrationSystemComponent::ROS2ScriptIntegrationSystemComponent()
    {
        if (ROS2ScriptIntegrationInterface::Get() == nullptr)
        {
            ROS2ScriptIntegrationInterface::Register(this);
        }
        if (PublisherRequesIterface::Get() == nullptr)
        {
            PublisherRequesIterface::Register(this);
        }
    }

    ROS2ScriptIntegrationSystemComponent::~ROS2ScriptIntegrationSystemComponent()
    {
        if (ROS2ScriptIntegrationInterface::Get() == this)
        {
            ROS2ScriptIntegrationInterface::Unregister(this);
        }
        if (PublisherRequesIterface::Get() == this)
        {
            PublisherRequesIterface::Unregister(this);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::Init()
    {
    }

    void ROS2ScriptIntegrationSystemComponent::Activate()
    {
        ROS2ScriptIntegrationRequestBus::Handler::BusConnect();
        PublisherRequestBus::Handler::BusConnect();
        SubscriberRequestBus::Handler::BusConnect();
    }

    void ROS2ScriptIntegrationSystemComponent::Deactivate()
    {
        std::lock_guard<AZStd::shared_mutex> lock1(m_publisherMapMutex);
        m_publishers.clear();
        std::lock_guard<AZStd::shared_mutex> lock2(m_subscribersMapMutex);
        m_subscribers.clear();
        SubscriberRequestBus::Handler::BusDisconnect();
        PublisherRequestBus::Handler::BusDisconnect();
        ROS2ScriptIntegrationRequestBus::Handler::BusDisconnect();
    }

    // Publishers overrides ...
    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgsString(const AZStd::string& topicName, const AZStd::string& value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::String>(topicName);
        if (publisher)
        {
            std_msgs::msg::String message;
            message.data = std::string(value.c_str());
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgEmpty(const AZStd::string& topicName)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Empty>(topicName);
        if (publisher)
        {
            publisher->publish(std_msgs::msg::Empty());
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgUInt32(const AZStd::string& topicName, const uint32_t value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::UInt32>(topicName);
        if (publisher)
        {
            auto message = std_msgs::msg::UInt32();
            message.data = value;
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgInt32(const AZStd::string& topicName, const int32_t value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Int32>(topicName);
        if (publisher)
        {
            std_msgs::msg::Int32 message;
            message.data = value;
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgFloat32(const AZStd::string& topicName, const float value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Float32>(topicName);
        if (publisher)
        {
            std_msgs::msg::Float32 message;
            message.data = value;
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgBool(const AZStd::string& topicName, const bool value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Bool>(topicName);
        if (publisher)
        {
            std_msgs::msg::Bool message;
            message.data = value;
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgsTwist(
        const AZStd::string& topicName, const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Twist>(topicName);
        if (publisher)
        {
            geometry_msgs::msg::Twist message;
            message.linear.x = linear.GetX();
            message.linear.y = linear.GetY();
            message.linear.z = linear.GetZ();
            message.angular.x = angular.GetX();
            message.angular.y = angular.GetY();
            message.angular.z = angular.GetZ();
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgTransform(const AZStd::string& topicName, const AZ::Transform& transform)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Transform>(topicName);
        if (publisher)
        {
            geometry_msgs::msg::Transform message;
            message.translation.x = transform.GetTranslation().GetX();
            message.translation.y = transform.GetTranslation().GetY();
            message.translation.z = transform.GetTranslation().GetZ();
            message.rotation.x = transform.GetRotation().GetX();
            message.rotation.y = transform.GetRotation().GetY();
            message.rotation.z = transform.GetRotation().GetZ();
            message.rotation.w = transform.GetRotation().GetW();
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgVector3(const AZStd::string& topicName, const AZ::Vector3& vector)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Vector3>(topicName);
        if (publisher)
        {
            geometry_msgs::msg::Vector3 message;
            message.x = vector.GetX();
            message.y = vector.GetY();
            message.z = vector.GetZ();
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgQuaternion(
        const AZStd::string& topicName, const AZ::Quaternion& quaternion)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Quaternion>(topicName);
        if (publisher)
        {
            geometry_msgs::msg::Quaternion message;
            message.x = quaternion.GetX();
            message.y = quaternion.GetY();
            message.z = quaternion.GetZ();
            message.w = quaternion.GetW();
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgPoint32(const AZStd::string& topicName, const AZ::Vector3& point)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Point32>(topicName);
        if (publisher)
        {
            geometry_msgs::msg::Point32 message;
            message.x = point.GetX();
            message.y = point.GetY();
            message.z = point.GetZ();
            publisher->publish(message);
        }
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgPoseStamped(
        const AZStd::string& topicName, const AZStd::string& frame, const AZ::Transform& transform)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::PoseStamped>(topicName);
        if (publisher)
        {
            geometry_msgs::msg::PoseStamped message;
            message.header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
            message.header.frame_id = std::string(frame.c_str());
            message.pose.position.x = transform.GetTranslation().GetX();
            message.pose.position.y = transform.GetTranslation().GetY();
            message.pose.position.z = transform.GetTranslation().GetZ();
            message.pose.orientation.x = transform.GetRotation().GetX();
            message.pose.orientation.y = transform.GetRotation().GetY();
            message.pose.orientation.z = transform.GetRotation().GetZ();
            message.pose.orientation.w = transform.GetRotation().GetW();
            publisher->publish(message);
        }
    }

    // SubscriberRequests overrides ...
    void ROS2ScriptIntegrationSystemComponent::SubscribeToStdMsgBool(const AZStd::string& topicName)
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

    void ROS2ScriptIntegrationSystemComponent::SubscribeToSensorMsgJoy(const AZStd::string& topicName)
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

    void ROS2ScriptIntegrationSystemComponent::SubscribeToGeometryMsgPoseStamped(const AZStd::string& topicName)
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

    void ROS2ScriptIntegrationSystemComponent::SubscribeToString(const AZStd::string& topicName)
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
    void ROS2ScriptIntegrationSystemComponent::SubscribeToFloat32(const AZStd::string& topicName)
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

    void ROS2ScriptIntegrationSystemComponent::SubscribeToUInt32(const AZStd::string& topicName)
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

    void ROS2ScriptIntegrationSystemComponent::SubscribeToInt32(const AZStd::string& topicName)
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

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<MessageType>> ROS2ScriptIntegrationSystemComponent::GetOrCreatePublisher(
        const AZStd::string& topicName)
    {
        std::lock_guard<AZStd::shared_mutex> lock(m_publisherMapMutex);
        auto it = m_publishers.find(topicName);
        if (it != m_publishers.end())
        {
            return std::dynamic_pointer_cast<rclcpp::Publisher<MessageType>>(it->second);
        }
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            // create a new publisher
            std::shared_ptr<rclcpp::Publisher<MessageType>> publisher =
                ros2Node->create_publisher<MessageType>(std::string(topicName.c_str()), 10);
            m_publishers[topicName] = publisher;
            return publisher;
        }
        return nullptr;
    }

} // namespace ROS2ScriptIntegration
