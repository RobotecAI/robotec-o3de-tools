#pragma once

#include "SplineSubscriberConfig.h"
#include <AzCore/Component/Component.h>
#include <ROS2/ROS2Bus.h>
#include <SplineTools/SplineToolsTypeIds.h>
#include <nav_msgs/msg/path.hpp>

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
        void OnSplineReceived(const nav_msgs::msg::Path& msg);
        SplineSubscriberConfiguration m_config;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_subscription;
    };

} // namespace SplineTools
