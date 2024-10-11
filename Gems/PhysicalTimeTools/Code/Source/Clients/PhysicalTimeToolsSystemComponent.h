
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <PhysicalTimeTools/PhysicalTimeToolsBus.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <PhysX/Configuration/PhysXConfiguration.h>

namespace PhysicalTimeTools
{
    class PhysicalTimeToolsSystemComponent
        : public AZ::Component
        , protected PhysicalTimeToolsRequestBus::Handler
        , public AZ::TickBus::Handler
        , public AzFramework::InputChannelEventListener
    {
    public:
        AZ_COMPONENT_DECL(PhysicalTimeToolsSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        PhysicalTimeToolsSystemComponent();
        ~PhysicalTimeToolsSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // PhysicalTimeToolsRequestBus interface implementation

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

        // AzFramework::InputChannelEventListener overrides ...
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

        bool m_isPaused = true;
        float m_timeScale = 1.0f;
        AZStd::optional<PhysX::PhysXSystemConfiguration> m_physicsInitialConfig;
        bool m_configDirty = false;
        PhysX::PhysXSystemConfiguration m_physicsConfig;
    };

} // namespace PhysicalTimeTools
