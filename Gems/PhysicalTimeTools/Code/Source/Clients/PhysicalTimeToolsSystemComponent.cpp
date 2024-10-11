
#include "PhysicalTimeToolsSystemComponent.h"

#include <PhysicalTimeTools/PhysicalTimeToolsTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Input/Devices/Keyboard/InputDeviceKeyboard.h>


namespace PhysicalTimeTools
{
    AZ_COMPONENT_IMPL(PhysicalTimeToolsSystemComponent, "PhysicalTimeToolsSystemComponent",
        PhysicalTimeToolsSystemComponentTypeId);

    void PhysicalTimeToolsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PhysicalTimeToolsSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void PhysicalTimeToolsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PhysicalTimeToolsService"));
    }

    void PhysicalTimeToolsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PhysicalTimeToolsService"));
    }

    void PhysicalTimeToolsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void PhysicalTimeToolsSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    PhysicalTimeToolsSystemComponent::PhysicalTimeToolsSystemComponent()
    {
        if (PhysicalTimeToolsInterface::Get() == nullptr)
        {
            PhysicalTimeToolsInterface::Register(this);
        }
    }

    PhysicalTimeToolsSystemComponent::~PhysicalTimeToolsSystemComponent()
    {
        if (PhysicalTimeToolsInterface::Get() == this)
        {
            PhysicalTimeToolsInterface::Unregister(this);
        }
    }

    void PhysicalTimeToolsSystemComponent::Init()
    {
    }

    void PhysicalTimeToolsSystemComponent::Activate()
    {
        PhysicalTimeToolsRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
        InputChannelEventListener::Connect();
    }

    void PhysicalTimeToolsSystemComponent::Deactivate()
    {
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
        PhysicalTimeToolsRequestBus::Handler::BusDisconnect();
        InputChannelEventListener::Disconnect();
    }

    void PhysicalTimeToolsSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system.");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface.");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        if (defaultSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return;
        }
        AzPhysics::Scene* scene = sceneInterface->GetScene(defaultSceneHandle);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene pointer.");
        scene->SetEnabled(!m_isPaused);

        if (!m_physicsInitialConfig)
        {
            auto *physicsSystemConfigurationPtr = physicsSystem->GetConfiguration();
            auto *physicsSystemConfiguration =  azdynamic_cast<const PhysX::PhysXSystemConfiguration*>(physicsSystemConfigurationPtr);
            AZ_Assert(physicsSystemConfiguration, "Invalid physics system configuration pointer, new Physics system in O3DE????");
            m_physicsInitialConfig = *physicsSystemConfiguration;
            m_physicsConfig = *physicsSystemConfiguration;
        }
        else
        {
            if (m_configDirty)
            {
                m_physicsConfig.m_maxTimestep = m_physicsInitialConfig->m_maxTimestep * m_timeScale;
                physicsSystem->UpdateConfiguration(&m_physicsConfig, true);
                m_configDirty = false;
            }
        }

    }

    bool PhysicalTimeToolsSystemComponent::OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputDeviceId& deviceId = inputChannel.GetInputDevice().GetInputDeviceId();

        if (AzFramework::InputDeviceKeyboard::IsKeyboardDevice(deviceId) && inputChannel.IsStateBegan())
        {
            if (inputChannel.GetInputChannelId() == AzFramework::InputDeviceKeyboard::Key::AlphanumericP)
            {
                    m_isPaused = !m_isPaused;
                    AZ::TickBus::Handler::BusConnect();
                    return true;
            }
        }
        if (AzFramework::InputDeviceKeyboard::IsKeyboardDevice(deviceId) && inputChannel.IsStateBegan())
        {
            if (inputChannel.GetInputChannelId() == AzFramework::InputDeviceKeyboard::Key::NavigationPageUp)
            {
                m_timeScale *=1.1f;
                AZ_Printf("PhysicalTimeTools", "TimeScale: %f", m_timeScale);
                m_configDirty = true;
                return true;
            }
            if (inputChannel.GetInputChannelId() == AzFramework::InputDeviceKeyboard::Key::NavigationPageDown)
            {
                m_timeScale *=0.9f;
                AZ_Printf("PhysicalTimeTools", "TimeScale: %f", m_timeScale);
                m_configDirty = true;
                return true;
            }

        }

        return false;
    }


} // namespace PhysicalTimeTools
