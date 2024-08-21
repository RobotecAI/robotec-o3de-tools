
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include "ROS2/ROS2Bus.h"
#include "std_msgs/msg/string.hpp"
#include <AzFramework/Components/ConsoleBus.h>

namespace ExposeConsoleToRos
{
    class ExposeConsoleToRosSystemComponent
        : public AZ::Component
        , public AzFramework::ConsoleNotificationBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ExposeConsoleToRosSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        ExposeConsoleToRosSystemComponent() = default;
        ~ExposeConsoleToRosSystemComponent() = default;

    protected:
        // AzFramework::ConsoleNotificationBus::Handler
        void OnConsoleCommandExecuted(const char* command) override;

        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> m_consoleSubscription;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> m_consolePublisher;
    };

} // namespace ExposeConsoleToRos
