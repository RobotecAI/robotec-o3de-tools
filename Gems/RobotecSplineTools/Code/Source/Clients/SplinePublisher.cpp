#include "SplinePublisher.h"

#include "ROS2/Utilities/ROS2Names.h"

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>

namespace SplineTools
{
    SplinePublisherConfiguration::SplinePublisherConfiguration()
    {
        m_TopicConfig.m_type = "nav_msgs::msg::Path";
        m_TopicConfig.m_topic = "spline_path";
    }

    void SplinePublisherConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplinePublisherConfiguration>()->Version(0)->Field(
                "m_topicName", &SplinePublisherConfiguration::m_TopicConfig);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<SplinePublisherConfiguration>(
                        "SplinePublisherConfiguration", "Configuration for the SplineSubscriber component")
                    ->ClassElement(AZ::Edit::ClassElements::Group, "SplineSubscriber Configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SplinePublisherConfiguration::m_TopicConfig, "Topic Config", "Topic Config");
            }
        }
    }

    void SplinePublisher::Reflect(AZ::ReflectContext* context)
    {
        SplinePublisherConfiguration::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplinePublisher, AZ::Component>()->Version(0)->Field("m_config", &SplinePublisher::m_config);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SplinePublisher>("SplinePathPublisher", "Enables to publish spline as a ros2 path.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "SplinePathPublisher")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplinePublisher::m_config,
                        "Configuration",
                        "Configuration for the SplinePathPublisher component");
            }
        }
    }

    SplinePublisher::SplinePublisher(const SplinePublisherConfiguration& config)
        : m_config(config)
    {
    }

    void SplinePublisher::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("SplineService", 0x2b674d3c));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void SplinePublisher::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (ros2Node)
        {
            m_publisher =
                ros2Node->create_publisher<nav_msgs::msg::Path>(m_config.m_TopicConfig.m_topic.data(), m_config.m_TopicConfig.GetQoS());

            AZ::TickBus::Handler::BusConnect();
        }
    }

    void SplinePublisher::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_publisher.reset();
    }

    void SplinePublisher::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        AZ_UNUSED(deltaTime);
        AZ_UNUSED(time);

        PublishSplineAsPath();
    }

    void SplinePublisher::PublishSplineAsPath() const
    {
        if (!m_publisher)
        {
            return;
        }

        auto* ros2Frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        nav_msgs::msg::Path pathMessage;
        pathMessage.header.frame_id = ros2Frame->GetFrameID().data(); // Set an appropriate frame ID (as per your use case)
        pathMessage.header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();

        // Generate the path based on the spline (retrieve spline data from the SplineService)
        AZStd::vector<AZ::Vector3> splinePoints;
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePoints, GetEntityId(), &LmbrCentral::SplineComponentRequests::GetSpline);

        for (const auto& point : splinePoints)
        {
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.pose.position.x = point.GetX();
            poseStamped.pose.position.y = point.GetY();
            poseStamped.pose.position.z = point.GetZ();
            pathMessage.poses.push_back(poseStamped);
        }

        // Publish the message
        m_publisher->publish(pathMessage);
    }
} // namespace SplineTools