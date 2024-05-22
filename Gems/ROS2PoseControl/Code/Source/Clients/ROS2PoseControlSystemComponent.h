
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2PoseControl/ROS2PoseControlBus.h>

namespace ROS2PoseControl
{
    class ROS2PoseControlSystemComponent
        : public AZ::Component
        , protected ROS2PoseControlRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ROS2PoseControlSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROS2PoseControlSystemComponent();
        ~ROS2PoseControlSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ROS2PoseControlRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        ////////////////////////////////////////////////////////////////////////
    };

} // namespace ROS2PoseControl
