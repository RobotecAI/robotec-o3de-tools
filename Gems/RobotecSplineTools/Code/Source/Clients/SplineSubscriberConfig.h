#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <SplineTools/SplineToolsTypeIds.h>

namespace SplineTools
{
    //! A structure capturing configuration of a IMU sensor.
    struct SplineSubscriberConfiguration
    {
        SplineSubscriberConfiguration();

        AZ_TYPE_INFO(SplineSubscriberConfiguration, SplineSubscriberConfigTypeId);
        static void Reflect(AZ::ReflectContext* context);
        ROS2::TopicConfiguration m_topic{ rclcpp::ServicesQoS() };
        bool m_allowWGS84{ true }; //! Allow WGS84 coordinates.
        bool m_resetOnActivation{ true }; //! Reset entity's spline on activation.
        AZStd::string m_startOffsetTag; //! Tag of the entity to use for the spline points offset.
    };
} // namespace SplineTools
