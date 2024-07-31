
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <RobotecSpectatorCamera/RobotecSpectatorCameraBus.h>

namespace RobotecSpectatorCamera
{
    class RobotecSpectatorCameraSystemComponent
        : public AZ::Component
        , protected RobotecSpectatorCameraRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(RobotecSpectatorCameraSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        RobotecSpectatorCameraSystemComponent();
        ~RobotecSpectatorCameraSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // RobotecSpectatorCameraRequestBus interface implementation

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

} // namespace RobotecSpectatorCamera
