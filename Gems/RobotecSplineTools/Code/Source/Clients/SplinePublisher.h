#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <SplineTools/SplineToolsTypeIds.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace SplineTools
{
    struct SplinePublisherConfiguration
    {
        AZ_TYPE_INFO(SplinePublisherConfiguration, SplinePublisherConfigTypeId);
        static void Reflect(AZ::ReflectContext* context);

        ROS2::TopicConfiguration m_TopicConfig{ rclcpp::ServicesQoS() };
        int m_updateFrequency = 10;
        SplinePublisherConfiguration();
    };

    class SplinePublisher final
        : public AZ::Component
        , protected AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(SplinePublisher, SplinePublisherComponentTypeId);

        static void Reflect(AZ::ReflectContext* context);

        SplinePublisher() = default;
        ~SplinePublisher() override = default;
        explicit SplinePublisher(SplinePublisherConfiguration config);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

    protected:
        // AZ::TickBus::Handler interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        void PublishSplineAsPath() const;

        SplinePublisherConfiguration m_config;
        int m_frameNumber = 0;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_publisher;
        ROS2::ROS2FrameComponent* m_ros2FramePtr = nullptr;
    };
} // namespace SplineTools
