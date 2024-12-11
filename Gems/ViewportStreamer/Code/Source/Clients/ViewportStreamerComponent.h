
#pragma once

#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <ViewportStreamer/ViewportStreamerTypeIds.h>

namespace ViewportStreamer
{
    class ViewportStreamerComponent : public ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>
    {
    public:
        AZ_COMPONENT_DECL(ViewportStreamerComponent);

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        ViewportStreamerComponent();
        ~ViewportStreamerComponent();

        AZStd::string m_frameName;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_imagePublisher;
        ROS2::TopicConfiguration m_imagePublisherTopic;

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        void FrequencyTick();
        void RequestMessagePublication(const AZStd::vector<AZStd::string>& passHierarchy, const std_msgs::msg::Header& header);
        void RequestFrame(
            const AZStd::vector<AZStd::string>& passHierarchy,
            AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);
    };

} // namespace ViewportStreamer
