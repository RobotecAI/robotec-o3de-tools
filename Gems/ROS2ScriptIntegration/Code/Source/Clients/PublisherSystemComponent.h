
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
    class PublisherSystemComponent
        : public AZ::Component
        , protected PublisherRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(PublisherSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        PublisherSystemComponent();

        ~PublisherSystemComponent();

    protected:
        // ROS2ScriptIntegrationRequestBus::Handler overrides ...
        void PublishStdMsgsString(const AZStd::string& topicName, const AZStd::string& value) override;

        void PublishStdMsgEmpty(const AZStd::string& topicName) override;

        void PublishStdMsgUInt32(const AZStd::string& topicName, const uint32_t value) override;

        void PublishStdMsgInt32(const AZStd::string& topicName, const int32_t value) override;

        void PublishStdMsgFloat32(const AZStd::string& topicName, const float value) override;

        void PublishStdMsgBool(const AZStd::string& topicName, const bool value) override;

        void PublishGeometryMsgsTwist(const AZStd::string& topicName, const AZ::Vector3& linear, const AZ::Vector3& angular) override;

        void PublishGeometryMsgTransform(const AZStd::string& topicName, const AZ::Transform& transform) override;

        void PublishGeometryMsgVector3(const AZStd::string& topicName, const AZ::Vector3& vector) override;

        void PublishGeometryMsgQuaternion(const AZStd::string& topicName, const AZ::Quaternion& quaternion) override;

        void PublishGeometryMsgPoint32(const AZStd::string& topicName, const AZ::Vector3& point) override;

        void PublishGeometryMsgPoseStamped(
            const AZStd::string& topicName, const AZStd::string& frame, const AZ::Transform& transform) override;

        void PublishAckermannDriveMsg(
            const AZStd::string& topicName, float steeringAngle, float steeringVelocity, float speed, float acceleration, float jerk)
            override;

        // AZ::Component overrides ...
        void Init() override;

        void Activate() override;

        void Deactivate() override;

    private:
        AZStd::shared_mutex m_publisherMapMutex;
        AZStd::unordered_map<AZStd::string, std::shared_ptr<rclcpp::PublisherBase>> m_publishers;

        //! This function will return a publisher for a given topic name. If the publisher does not exist, it will be created.
        //! @tparam MessageType the type of message to publish
        //! @param topicName the name of the topic to publish to
        //! @return a publisher for the given topic name, nullptr is returned if the publisher could not be created
        template<typename MessageType>
        std::shared_ptr<rclcpp::Publisher<MessageType>> GetOrCreatePublisher(const AZStd::string& topicName);

        template<typename MessageType>
        void PublishMessage(AZStd::string topicName, const MessageType& message);
    };

} // namespace ROS2ScriptIntegration
