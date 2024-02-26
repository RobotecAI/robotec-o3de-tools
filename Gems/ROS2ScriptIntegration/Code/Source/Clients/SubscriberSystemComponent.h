
#pragma once

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/parallel/shared_mutex.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationBus.h>
#include <ROS2ScriptIntegration/ROS2ScriptPublisherBus.h>
#include <ROS2ScriptIntegration/ROS2ScriptSubscriberBus.h>

namespace ROS2ScriptIntegration
{
    class SubscriberSystemComponent
        : public AZ::Component
        , protected SubscriberRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(SubscriberSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        SubscriberSystemComponent();

        ~SubscriberSystemComponent();

    protected:
        // SubscriberRequestBus overrides ...
        void SubscribeToStdMsgBool(const AZStd::string& topicName) override;
        void SubscribeToSensorMsgJoy(const AZStd::string& topicName) override;
        void SubscribeToGeometryMsgPoseStamped(const AZStd::string& topicName) override;
        void SubscribeToString(const AZStd::string& topicName) override;
        void SubscribeToFloat32(const AZStd::string& topicName) override;
        void SubscribeToUInt32(const AZStd::string& topicName) override;
        void SubscribeToInt32(const AZStd::string& topicName) override;

        // AZ::Component overrides ...
        void Init() override;

        void Activate() override;

        void Deactivate() override;

    private:
        AZStd::shared_mutex m_subscribersMapMutex;
        AZStd::unordered_map<AZStd::string, std::shared_ptr<rclcpp::SubscriptionBase>> m_subscribers;

        //! Helper function to subscribe to a topic and store the subscription in the subscribers map
        //! @tparam MessageType The type of message to subscribe to
        //! @param topicName The name of the topic to subscribe to
        //! @param callback The callback to call when a message is received
        template<typename MessageType>
        void SubscribeToTopic(const AZStd::string& topicName, const std::function<void(const MessageType&)>& callback);
    };

} // namespace ROS2ScriptIntegration
