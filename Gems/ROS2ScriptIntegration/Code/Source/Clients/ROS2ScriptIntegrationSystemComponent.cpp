
#include "ROS2ScriptIntegrationSystemComponent.h"

#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ScriptIntegrationSystemComponent, AZ::Component>()->Version(0);
        }
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<PublisherRequestBus>("PublisherRequestBus")
                ->Attribute(AZ::Script::Attributes::Category, "ROS2")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "ROS2")
                ->Event(
                    "PublishStdMsgsString", &PublisherRequestBus::Events::PublishStdMsgsString, { { { "Topic", "" }, { "Value", "" } } })
                ->Event("PublishStdMsgEmpty", &PublisherRequestBus::Events::PublishStdMsgEmpty, { { { "Topic", "" } } })
                ->Event("PublishStdMsgUInt32", &PublisherRequestBus::Events::PublishStdMsgUInt32, { { { "Topic", "" }, { "Value", "" } } })
                ->Event("PublishStdMsgInt32", &PublisherRequestBus::Events::PublishStdMsgInt32, { { { "Topic", "" }, { "Value", "" } } })
                ->Event(
                    "PublishStdMsgFloat32", &PublisherRequestBus::Events::PublishStdMsgFloat32, { { { "Topic", "" }, { "Value", "" } } })
                ->Event("PublishStdMsgBool", &PublisherRequestBus::Events::PublishStdMsgBool, { { { "Topic", "" }, { "Value", "" } } })
                ->Event(
                    "PublishGeometryMsgsTwist",
                    &PublisherRequestBus::Events::PublishGeometryMsgsTwist,
                    { { { "Topic", "" }, { "Linear", "" }, { "Angular", "" } } })
                ->Event(
                    "PublishGeometryMsgTransform",
                    &PublisherRequestBus::Events::PublishGeometryMsgTransform,
                    { { { "Topic", "" }, { "Transform", "" } } })
                ->Event(
                    "PublishGeometryMsgVector3",
                    &PublisherRequestBus::Events::PublishGeometryMsgVector3,
                    { { { "Topic", "" }, { "Vector", "" } } })
                ->Event(
                    "PublishGeometryMsgQuaternion",
                    &PublisherRequestBus::Events::PublishGeometryMsgQuaternion,
                    { { { "Topic", "" }, { "Quaternion", "" } } })
                ->Event(
                    "PublishGeometryMsgPoint32",
                    &PublisherRequestBus::Events::PublishGeometryMsgPoint32,
                    { { { "Topic", "" }, { "Point", "" } } });
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
    }

    void ROS2ScriptIntegrationSystemComponent::Deactivate()
    {
        PublisherRequestBus::Handler::BusDisconnect();
        ROS2ScriptIntegrationRequestBus::Handler::BusDisconnect();
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgsString(const AZStd::string& topicName, const AZStd::string& value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::String>(topicName);
        if (publisher)
        {
            auto message = std_msgs::msg::String();
            message.data = std::string(value.c_str());
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgEmpty(const AZStd::string& topicName)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Empty>(topicName);
        if (publisher)
        {
            auto message = std_msgs::msg::Empty();
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgUInt32(const AZStd::string& topicName, const uint32_t value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::UInt32>(topicName);
        if (publisher)
        {
            auto message = std_msgs::msg::UInt32();
            message.data = value;
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgInt32(const AZStd::string& topicName, const int32_t value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Int32>(topicName);
        if (publisher)
        {
            auto message = std_msgs::msg::Int32();
            message.data = value;
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgFloat32(const AZStd::string& topicName, const float value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Float32>(topicName);
        if (publisher)
        {
            auto message = std_msgs::msg::Float32();
            message.data = value;
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishStdMsgBool(const AZStd::string& topicName, const bool value)
    {
        auto publisher = GetOrCreatePublisher<std_msgs::msg::Bool>(topicName);
        if (publisher)
        {
            auto message = std_msgs::msg::Bool();
            message.data = value;
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgsTwist(
        const AZStd::string& topicName, const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Twist>(topicName);
        if (publisher)
        {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = linear.GetX();
            message.linear.y = linear.GetY();
            message.linear.z = linear.GetZ();
            message.angular.x = angular.GetX();
            message.angular.y = angular.GetY();
            message.angular.z = angular.GetZ();
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgTransform(const AZStd::string& topicName, const AZ::Transform& transform)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Transform>(topicName);
        if (publisher)
        {
            auto message = geometry_msgs::msg::Transform();
            message.translation.x = transform.GetTranslation().GetX();
            message.translation.y = transform.GetTranslation().GetY();
            message.translation.z = transform.GetTranslation().GetZ();
            message.rotation.x = transform.GetRotation().GetX();
            message.rotation.y = transform.GetRotation().GetY();
            message.rotation.z = transform.GetRotation().GetZ();
            message.rotation.w = transform.GetRotation().GetW();
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgVector3(const AZStd::string& topicName, const AZ::Vector3& vector)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Vector3>(topicName);
        if (publisher)
        {
            auto message = geometry_msgs::msg::Vector3();
            message.x = vector.GetX();
            message.y = vector.GetY();
            message.z = vector.GetZ();
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgQuaternion(
        const AZStd::string& topicName, const AZ::Quaternion& quaternion)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Quaternion>(topicName);
        if (publisher)
        {
            auto message = geometry_msgs::msg::Quaternion();
            message.x = quaternion.GetX();
            message.y = quaternion.GetY();
            message.z = quaternion.GetZ();
            message.w = quaternion.GetW();
            publisher->publish(message);
            return;
        }
        return;
    }

    void ROS2ScriptIntegrationSystemComponent::PublishGeometryMsgPoint32(const AZStd::string& topicName, const AZ::Vector3& point)
    {
        auto publisher = GetOrCreatePublisher<geometry_msgs::msg::Point32>(topicName);
        if (publisher)
        {
            auto message = geometry_msgs::msg::Point32();
            message.x = point.GetX();
            message.y = point.GetY();
            message.z = point.GetZ();
            publisher->publish(message);
            return;
        }
        return;
    }

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<MessageType>>

    ROS2ScriptIntegrationSystemComponent::GetOrCreatePublisher(const AZStd::string& topicName)
    {
        std::lock_guard<AZStd::mutex> lock(m_publisherMapMutex);
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

    // Explicit template instantiation for all message types
    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt32>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Point32>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Quaternion>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Transform>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>

    GetOrCreatePublisher(const AZStd::string& topicName);

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Vector3>>

    GetOrCreatePublisher(const AZStd::string& topicName);

} // namespace ROS2ScriptIntegration
