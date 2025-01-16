
#pragma once

#include <AzCore/Component/Component.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <ROS2/Sensor/Events/EventSourceAdapter.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ViewportStreamer/ViewportStreamerTypeIds.h>

namespace ViewportStreamer
{
    class ViewportStreamerSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT_DECL(ViewportStreamerSystemComponent);

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        ViewportStreamerSystemComponent();
        ~ViewportStreamerSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        void FrequencyTick();
        void RequestMessagePublication(const AZStd::vector<AZStd::string>& passHierarchy, const std_msgs::msg::Header& header);

        ROS2::EventSourceAdapter<ROS2::TickBasedSource> m_eventSourceAdapter;
        typename ROS2::TickBasedSource::AdaptedEventHandlerType m_adaptedEventHandler;

        AZStd::string m_frameName;
        AZ::u64 m_frequency;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_imagePublisher;
        ROS2::TopicConfiguration m_imagePublisherTopic;
    };

} // namespace ViewportStreamer
