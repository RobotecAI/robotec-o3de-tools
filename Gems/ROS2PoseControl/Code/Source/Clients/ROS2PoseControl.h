#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Script/ScriptTimePoint.h>
#include <AzCore/std/containers/vector.h>
#include <ImGuiBus.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <AzCore/Math/Transform.h>
#include <ROS2PoseControl/ROS2PoseControlConfiguration.h>

#include "AzCore/Time/ITime.h"
#include "ROS2/Sensor/ROS2SensorComponentBase.h"

namespace ROS2PoseControl {
    class ROS2PoseControl
            : public AZ::Component
              , public AZ::TickBus::Handler
              , public ImGui::ImGuiUpdateListenerBus::Handler {
    public:
        AZ_COMPONENT(ROS2PoseControl, "{75e9cdd4-e460-4bea-b9f6-c673f820fb4c}", AZ::Component);

        ROS2PoseControl();

        ~ROS2PoseControl() = default;

        void Activate() override;

        void Deactivate() override;

        static void Reflect(AZ::ReflectContext *context);

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType &required);

        void SetIsTracking(bool isTracking);

        void SwitchToPoseMessages(ROS2::TopicConfiguration topicConfiguration);

        void SwitchToTF2(AZStd::string targetFrame, AZStd::string referenceFrame);

    private:
        void OnImGuiUpdate() override;

        [[nodiscard]] AZ::Outcome<AZ::Transform, const char *> GetCurrentTransformViaTF2() const;

        void OnTopicConfigurationChanged();

        void OnIsTrackingChanged();

        void OnTrackingModeChanged();

        bool m_isTracking = false;
        ROS2PoseControlConfiguration m_configuration;

        // Pose Messages Tracking
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > m_poseSubscription;

        // TF2 Tracking
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    };
} // namespace PerceptionSimulation
