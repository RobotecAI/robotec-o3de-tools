
#pragma once

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/parallel/mutex.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationBus.h>
#include <ROS2ScriptIntegration/ROS2ScriptPublisherBus.h>

namespace ROS2ScriptIntegration
{
    class ROS2ScriptIntegrationSystemComponent
        : public AZ::Component
        , protected ROS2ScriptIntegrationRequestBus::Handler
        , protected PublisherRequestBus::Handler

    {
    public:
        AZ_COMPONENT_DECL(ROS2ScriptIntegrationSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROS2ScriptIntegrationSystemComponent();
        ~ROS2ScriptIntegrationSystemComponent();

    protected:
        // ROS2ScriptIntegrationRequestBus::Handler overrides ...
        void PublishStdMsgsString(const AZStd::string& topicName, const AZStd::string& value);
        void PublishStdMsgEmpty(const AZStd::string& topicName);
        void PublishStdMsgUInt32(const AZStd::string& topicName, const uint32_t value);
        void PublishStdMsgInt32(const AZStd::string& topicName, const int32_t value);
        void PublishStdMsgFloat32(const AZStd::string& topicName, const float value);
        void PublishStdMsgBool(const AZStd::string& topicName, const bool value);
        void PublishGeometryMsgsTwist(const AZStd::string& topicName, const AZ::Vector3& linear, const AZ::Vector3& angular);
        void PublishGeometryMsgTransform(const AZStd::string& topicName, const AZ::Transform& transform);
        void PublishGeometryMsgVector3(const AZStd::string& topicName, const AZ::Vector3& vector);
        void PublishGeometryMsgQuaternion(const AZStd::string& topicName, const AZ::Quaternion& quaternion);
        void PublishGeometryMsgPoint32(const AZStd::string& topicName, const AZ::Vector3& point);

        // AZ::Component overrides ...
        void Init() override;
        void Activate() override;
        void Deactivate() override;

    private:
        AZStd::mutex m_publisherMapMutex;
        AZStd::unordered_map<AZStd::string, std::shared_ptr<rclcpp::PublisherBase>> m_publishers;

        //! This function will return a publisher for a given topic name. If the publisher does not exist, it will be created.
        //! @tparam MessageType the type of message to publish
        //! @param topicName the name of the topic to publish to
        //! @return a publisher for the given topic name
        template<typename MessageType>
        std::shared_ptr<rclcpp::Publisher<MessageType>> GetOrCreatePublisher(const AZStd::string& topicName);
    };

} // namespace ROS2ScriptIntegration
