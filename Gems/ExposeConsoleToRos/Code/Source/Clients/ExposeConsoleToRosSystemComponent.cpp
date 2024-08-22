
#include "ExposeConsoleToRosSystemComponent.h"

#include <ExposeConsoleToRos/ExposeConsoleToRosTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace ExposeConsoleToRos
{

    AZ_COMPONENT_IMPL(ExposeConsoleToRosSystemComponent, "ExposeConsoleToRosSystemComponent", ExposeConsoleToRosSystemComponentTypeId);

    void ExposeConsoleToRosSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ExposeConsoleToRosSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ExposeConsoleToRosSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ExposeConsoleToRosService"));
    }

    void ExposeConsoleToRosSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ExposeConsoleToRosService"));
    }

    void ExposeConsoleToRosSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void ExposeConsoleToRosSystemComponent::Activate()
    {
#if defined(AZ_DEBUG_BUILD) || defined(AZ_PROFILE_BUILD)
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "ROS2 node is not available");
        m_consolePublisher = ros2Node->create_publisher<std_msgs::msg::String>("o3de_console_out", 10);
        m_consoleSubscription = ros2Node->create_subscription<std_msgs::msg::String>(
            "o3de_console_in",
            10,
            [](const std_msgs::msg::String::SharedPtr msg)
            {
                AZ_Info("ExposeConsoleToRos", "Received message: %s", msg->data.c_str());
                AzFramework::ConsoleRequestBus::Broadcast(&AzFramework::ConsoleRequests::ExecuteConsoleCommand, msg->data.c_str());
            });
        AzFramework::ConsoleNotificationBus::Handler::BusConnect();
#endif
    }

    void ExposeConsoleToRosSystemComponent::OnConsoleCommandExecuted(const char* command)
    {
        if (m_consolePublisher)
        {
            std_msgs::msg::String msg;
            msg.data = command;
            m_consolePublisher->publish(msg);
        }
    }

    void ExposeConsoleToRosSystemComponent::Deactivate()
    {
        if (AzFramework::ConsoleNotificationBus::Handler::BusIsConnected())
        {
            AzFramework::ConsoleNotificationBus::Handler::BusDisconnect();
        }
        m_consolePublisher.reset();
        m_consoleSubscription.reset();
    }

} // namespace ExposeConsoleToRos
