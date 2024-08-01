
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <DisableMainView/DisableMainViewBus.h>
namespace DisableMainView
{
    class DisableMainViewSystemComponent
        : public AZ::Component
        , private AZ::SystemTickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(DisableMainViewSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        DisableMainViewSystemComponent() = default;

        ~DisableMainViewSystemComponent() = default;

    protected:
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::SystemTickBus::Handler interface implementation
        void OnSystemTick() override;
    };

} // namespace DisableMainView
