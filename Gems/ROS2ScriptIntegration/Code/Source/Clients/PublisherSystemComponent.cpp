#include "PublisherSystemComponent.h"
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
    AZ_COMPONENT_IMPL(PublisherSystemComponent, "PublisherSystemComponent", PublisherSystemComponentTypeId);

    void PublisherSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        PublisherRequests::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PublisherSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void PublisherSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2ScriptIntegrationPublisherService"));
    }

    void PublisherSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2ScriptIntegrationPublisherService"));
    }

    void PublisherSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void PublisherSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    PublisherSystemComponent::PublisherSystemComponent()
    {
        if (PublisherRequesIterface::Get() == nullptr)
        {
            PublisherRequesIterface::Register(this);
        }
    }

    PublisherSystemComponent::~PublisherSystemComponent()
    {
        if (PublisherRequesIterface::Get() == this)
        {
            PublisherRequesIterface::Unregister(this);
        }
    }

    void PublisherSystemComponent::Init()
    {
    }

    void PublisherSystemComponent::Activate()
    {
        PublisherRequestBus::Handler::BusConnect();
    }

    void PublisherSystemComponent::Deactivate()
    {
        AZStd::lock_guard<AZStd::shared_mutex> lock1(m_publisherMapMutex);
        m_publishers.clear();
        PublisherRequestBus::Handler::BusDisconnect();
    }

    // Publishers overrides ...
    void PublisherSystemComponent::PublishStdMsgsString(const AZStd::string& topicName, const AZStd::string& value)
    {
        std_msgs::msg::String message;
        message.data = std::string(value.c_str());
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishStdMsgEmpty(const AZStd::string& topicName)
    {
        PublishMessage(topicName, std_msgs::msg::Empty());
    }

    void PublisherSystemComponent::PublishStdMsgUInt32(const AZStd::string& topicName, const uint32_t value)
    {
        std_msgs::msg::UInt32 message;
        message.data = value;
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishStdMsgInt32(const AZStd::string& topicName, const int32_t value)
    {
        std_msgs::msg::Int32 message;
        message.data = value;
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishStdMsgFloat32(const AZStd::string& topicName, const float value)
    {
        std_msgs::msg::Float32 message;
        message.data = value;
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishStdMsgBool(const AZStd::string& topicName, const bool value)
    {
        std_msgs::msg::Bool message;
        message.data = value;
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishGeometryMsgsTwist(
        const AZStd::string& topicName, const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        geometry_msgs::msg::Twist message;
        message.linear.x = linear.GetX();
        message.linear.y = linear.GetY();
        message.linear.z = linear.GetZ();
        message.angular.x = angular.GetX();
        message.angular.y = angular.GetY();
        message.angular.z = angular.GetZ();
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishGeometryMsgTransform(const AZStd::string& topicName, const AZ::Transform& transform)
    {
        geometry_msgs::msg::Transform message;
        message.translation.x = transform.GetTranslation().GetX();
        message.translation.y = transform.GetTranslation().GetY();
        message.translation.z = transform.GetTranslation().GetZ();
        message.rotation.x = transform.GetRotation().GetX();
        message.rotation.y = transform.GetRotation().GetY();
        message.rotation.z = transform.GetRotation().GetZ();
        message.rotation.w = transform.GetRotation().GetW();
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishGeometryMsgVector3(const AZStd::string& topicName, const AZ::Vector3& vector)
    {
        geometry_msgs::msg::Vector3 message;
        message.x = vector.GetX();
        message.y = vector.GetY();
        message.z = vector.GetZ();
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishGeometryMsgQuaternion(const AZStd::string& topicName, const AZ::Quaternion& quaternion)
    {
        geometry_msgs::msg::Quaternion message;
        message.x = quaternion.GetX();
        message.y = quaternion.GetY();
        message.z = quaternion.GetZ();
        message.w = quaternion.GetW();
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishGeometryMsgPoint32(const AZStd::string& topicName, const AZ::Vector3& point)
    {
        geometry_msgs::msg::Point32 message;
        message.x = point.GetX();
        message.y = point.GetY();
        message.z = point.GetZ();
        PublishMessage(topicName, message);
    }

    void PublisherSystemComponent::PublishGeometryMsgPoseStamped(
        const AZStd::string& topicName, const AZStd::string& frame, const AZ::Transform& transform)
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
        PublishMessage(topicName, message);
    }

    template<typename MessageType>
    std::shared_ptr<rclcpp::Publisher<MessageType>> PublisherSystemComponent::GetOrCreatePublisher(const AZStd::string& topicName)
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
            try
            {
                // create a new publisher
                std::shared_ptr<rclcpp::Publisher<MessageType>> publisher =
                    ros2Node->create_publisher<MessageType>(std::string(topicName.c_str()), 10);
                m_publishers[topicName] = publisher;
                return publisher;
            } catch (const std::exception& e)
            {
                AZ_Error("ROS2ScriptIntegration", false, "Failed to create publisher for topic %s: %s", topicName.c_str(), e.what());
            }
        }
        return nullptr;
    }
    template<typename MessageType>
    void PublisherSystemComponent::PublishMessage(AZStd::string topicName, const MessageType& message)
    {
        auto publisher = GetOrCreatePublisher<MessageType>(topicName);
        if (publisher)
        {
            publisher->publish(message);
        }
    }

} // namespace ROS2ScriptIntegration
