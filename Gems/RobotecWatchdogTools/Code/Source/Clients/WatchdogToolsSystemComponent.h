
#pragma once

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/parallel/shared_mutex.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <WatchdogTools/WatchdogToolsBus.h>

namespace WatchdogTools
{
    class WatchdogToolsSystemComponent
        : public AZ::Component
        , protected WatchdogToolsRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(WatchdogToolsSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        WatchdogToolsSystemComponent();

        ~WatchdogToolsSystemComponent();

    protected:
        // AZ::Component overrides ...
        void Init() override;

        void Activate() override;

        void Deactivate() override;
    };

} // namespace WatchdogTools
