
#pragma once

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/parallel/shared_mutex.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationBus.h>
#include <ROS2ScriptIntegration/ROS2ScriptPublisherBus.h>
#include <ROS2ScriptIntegration/ROS2ScriptSubscriberBus.h>

namespace ROS2ScriptIntegration
{
    class ROS2ScriptIntegrationSystemComponent
        : public AZ::Component
        , protected ROS2ScriptIntegrationRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ROS2ScriptIntegrationSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROS2ScriptIntegrationSystemComponent();

        ~ROS2ScriptIntegrationSystemComponent();

    protected:
        // AZ::Component overrides ...
        void Init() override;

        void Activate() override;

        void Deactivate() override;
    };

} // namespace ROS2ScriptIntegration
