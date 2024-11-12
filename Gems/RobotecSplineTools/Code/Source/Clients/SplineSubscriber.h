#pragma once

#include <AzCore/Component/Component.h>
#include <SplineTools/SplineToolsTypeIds.h>
#include "SplineSubscriberConfig.h"
#include <ROS2/ROS2Bus.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace SplineTools
{
    class SplineSubscriber : public AZ::Component
    {
    public:
        AZ_COMPONENT(SplineSubscriber, SplineSubscriberComponentTypeId, AZ::Component);

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;
    private:
        void OnSplineReceived(const geometry_msgs::msg::PoseStamped& msg);
        SplineSubscriberConfiguration m_config;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_subscription;
    };

} // namespace SplineTools
