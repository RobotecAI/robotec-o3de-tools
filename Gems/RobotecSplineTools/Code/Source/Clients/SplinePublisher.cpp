#include "SplinePublisher.h"

#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <utility>

namespace SplineTools
{
    SplinePublisherConfiguration::SplinePublisherConfiguration()
    {
        m_TopicConfig.m_type = "nav_msgs::msg::Path";
        m_TopicConfig.m_topic = "spline";
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
                editContext->Class<SplinePublisher>("SplinePathPublisher", "Enables to publish spline as a ROS 2 path.")
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

    SplinePublisher::SplinePublisher(SplinePublisherConfiguration config)
        : m_config(std::move(config))
    {
    }

    void SplinePublisher::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("SplineService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void SplinePublisher::Activate()
    {
        m_ros2FramePtr = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        if (!m_ros2FramePtr)
        {
            AZ_Warning("SplinePublisher::Activate", false, "ROS 2 frame component is not available!");
            return;
        }

        if (!m_ros2FramePtr->GetNamespace().empty())
        {
            m_config.m_TopicConfig.m_topic =
                AZStd::string::format("%s/%s", m_ros2FramePtr->GetNamespace().c_str(), m_config.m_TopicConfig.m_topic.c_str());
        }

        // Create the ROS2 Publisher
        if (const auto ros2Node = ROS2::ROS2Interface::Get()->GetNode())
        {
            m_publisher =
                ros2Node->create_publisher<nav_msgs::msg::Path>(m_config.m_TopicConfig.m_topic.c_str(), m_config.m_TopicConfig.GetQoS());

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
        PublishSplineAsPath();
    }

    void SplinePublisher::PublishSplineAsPath() const
    {
        if (!m_publisher)
        {
            return;
        }

        if (!m_ros2FramePtr)
        {
            AZ_Warning("SplinePublisher::PublishSplineAsPath", false, "ROS 2 frame component is not available!");
            return;
        }

        nav_msgs::msg::Path pathMessage;
        pathMessage.header.frame_id = m_ros2FramePtr->GetFrameID().data();
        pathMessage.header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();

        // Retrieve spline data
        AZStd::shared_ptr<AZ::Spline> spline;
        LmbrCentral::SplineComponentRequestBus::EventResult(spline, GetEntityId(), &LmbrCentral::SplineComponentRequests::GetSpline);

        if (!spline)
        {
            AZ_Warning("SplinePublisher::PublishSplineAsPath", false, "Spline not found. Cannot generate spline path.");
            return;
        }

        // Retrieve vertices from the spline
        const size_t vertexCount = spline->GetVertexCount();
        for (size_t i = 0; i < vertexCount; ++i)
        {
            const AZ::Vector3& vertex = spline->GetVertex(i);

            // Use emplace_back to construct PoseStamped in-place
            pathMessage.poses.emplace_back();
            auto& poseStamped = pathMessage.poses.back();

            // Set the pose values directly
            poseStamped.pose.position.x = vertex.GetX();
            poseStamped.pose.position.y = vertex.GetY();
            poseStamped.pose.position.z = vertex.GetZ();
        }

        m_publisher->publish(pathMessage);
    }
} // namespace SplineTools
