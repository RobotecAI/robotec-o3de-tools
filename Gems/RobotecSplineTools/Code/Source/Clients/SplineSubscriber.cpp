#include "SplineSubscriber.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace SplineTools
{

    void SplineSubscriber::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void SplineSubscriber::Reflect(AZ::ReflectContext* context)
    {
        SplineSubscriberConfiguration::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineSubscriber>()->Version(0)->Field("m_config", &SplineSubscriber::m_config);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SplineSubscriber>("SplineSubscriber", "Configuration for the SplineSubscriber component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "SplineSubscriber")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineSubscriber::m_config,
                        "Configuration",
                        "Configuration for the SplineSubscriber component");
            }
        }
    }

    void SplineSubscriber::Activate()
    {
        auto node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(node, "ROS 2 Node is not available");
        m_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            m_config.m_topic.m_topic.data(),
            m_config.m_topic.GetQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                OnSplineReceived(*msg);
            });
    }

    void SplineSubscriber::OnSplineReceived(const geometry_msgs::msg::PoseStamped& msg)
    {
        AZ_Printf("SplineSubscriber", "Received spline message");
    }
    void SplineSubscriber::Deactivate()
    {
        m_subscription.reset();
    }

} // namespace SplineTools