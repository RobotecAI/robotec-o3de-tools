
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
        using TypeName = std_msgs::msg::Bool;
        SubscribeToTopic<TypeName>(
            topicName,
            [topicName](const TypeName& msg)
            {
                SubscriberNotificationsBus::Event(topicName, &SubscriberNotificationsBus::Events::OnStdMsgBool, msg.data);
            });
    }

    void SubscriberSystemComponent::SubscribeToSensorMsgJoy(const AZStd::string& topicName)
    {
        using TypeName = sensor_msgs::msg::Joy;
        SubscribeToTopic<TypeName>(
            topicName,
            [topicName](const TypeName& msg)
            {
                TypeName cmsg{ msg };
                cmsg.buttons.resize(4);
                cmsg.axes.resize(4);
                SubscriberNotificationsBus::Event(
                    topicName,
                    &SubscriberNotificationsBus::Events::OnSensorMsgJoy,
                    cmsg.buttons[0],
                    cmsg.buttons[1],
                    cmsg.buttons[2],
                    cmsg.buttons[3],
                    cmsg.axes[0],
                    cmsg.axes[1],
                    cmsg.axes[2],
                    cmsg.axes[3]);
            });
    }

    void SubscriberSystemComponent::SubscribeToGeometryMsgPoseStamped(const AZStd::string& topicName)
    {
        using TypeName = geometry_msgs::msg::PoseStamped;
        SubscribeToTopic<TypeName>(
            topicName,
            [topicName](const TypeName& msg)
            {
                const auto& position = msg.pose.position;
                const auto& orientation = msg.pose.orientation;
                const auto transform = AZ::Transform::CreateFromQuaternionAndTranslation(
                    AZ::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
                    AZ::Vector3(position.x, position.y, position.z));
                const AZStd::string frameId(msg.header.frame_id.c_str());
                SubscriberNotificationsBus::Event(
                    topicName, &SubscriberNotificationsBus::Events::OnGeometryMsgPoseStamped, frameId, transform);
            });
    }

    void SubscriberSystemComponent::SubscribeToString(const AZStd::string& topicName)
    {
        using TypeName = std_msgs::msg::String;
        SubscribeToTopic<TypeName>(
            topicName,
            [topicName](const TypeName& msg)
            {
                const AZStd::string str(msg.data.c_str());
                SubscriberNotificationsBus::Event(topicName, &SubscriberNotificationsBus::Events::OnStdMsgString, str);
            });
    }

    void SubscriberSystemComponent::SubscribeToFloat32(const AZStd::string& topicName)
    {
        using TypeName = std_msgs::msg::Float32;
        SubscribeToTopic<TypeName>(
            topicName,
            [topicName](const TypeName& msg)
            {
                SubscriberNotificationsBus::Event(topicName, &SubscriberNotificationsBus::Events::OnStdMsgFloat32, msg.data);
            });
    }

    void SubscriberSystemComponent::SubscribeToUInt32(const AZStd::string& topicName)
    {
        using TypeName = std_msgs::msg::UInt32;
        SubscribeToTopic<TypeName>(
            topicName,
            [topicName](const TypeName& msg)
            {
                SubscriberNotificationsBus::Event(topicName, &SubscriberNotificationsBus::Events::OnStdMsgInt32, msg.data);
            });
    };

    void SubscriberSystemComponent::SubscribeToInt32(const AZStd::string& topicName)
    {
        using TypeName = std_msgs::msg::Int32;
        SubscribeToTopic<TypeName>(
            topicName,
            [topicName](const TypeName& msg)
            {
                SubscriberNotificationsBus::Event(topicName, &SubscriberNotificationsBus::Events::OnStdMsgInt32, msg.data);
            });
    };

    template<typename MessageType>
    void SubscriberSystemComponent::SubscribeToTopic(
        const AZStd::string& topicName, const std::function<void(const MessageType&)>& callback)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            auto subscriber = ros2Node->create_subscription<MessageType>(
                std::string(topicName.c_str()),
                10,
                [callback](typename MessageType::SharedPtr msg)
                {
                    callback(*msg);
                });
            std::lock_guard<AZStd::shared_mutex> lock(m_subscribersMapMutex);
            m_subscribers[topicName] = subscriber;
        }
    }

} // namespace ROS2ScriptIntegration
