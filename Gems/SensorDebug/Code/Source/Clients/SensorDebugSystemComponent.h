
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <ImGuiBus.h>
#include <PhysX/Configuration/PhysXConfiguration.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2/Sensor/SensorConfigurationRequestBus.h>
#include <ROS2/Sensor/SensorHelper.h>
#include <imgui/imgui.h>

namespace SensorDebug
{


    class SensorDebugSystemComponent
        : public AZ::Component
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(SensorDebugSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        SensorDebugSystemComponent() = default;
        ~SensorDebugSystemComponent() = default;

    protected:
        // ImGui::ImGuiUpdateListenerBus::Handler overrides...
        void OnImGuiUpdate() override;

        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

        void ClearSensors();

    private:
        // AZ::TickBus::Handler interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void UpdatePhysXConfig();
        void GetPhysXConfig();
        void Pause(bool isPaused);
        AzPhysics::Scene* GetScene();

        void FindSensorsWithBusAPI();
        void FindSensorsWithComponentAPI();
        void FindSensorsWithGivenType(const char* typeId);
        AZStd::vector<AZ::EntityComponentIdPair> m_sensorEntities;
        AZStd::unordered_map<AZ::EntityComponentIdPair, AZStd::vector<float>> m_sensorFrequencyHistory;
        AZStd::unordered_map<AZ::EntityComponentIdPair, AZStd::string> m_sensorNames;
        AZStd::unordered_map<AZ::EntityComponentIdPair, AZStd::string> m_entitiesNames;
        AZStd::unordered_map<AZ::EntityComponentIdPair, float> m_sensorFrequencies;
        AZStd::vector<float> m_appFrequencies;
        float m_maxFPS = 60.0f;
        int m_historySize = 1000;
        AzPhysics::Scene* m_scene = nullptr;
        PhysX::PhysXSystemConfiguration m_modifiedPhysXConfig;
        float m_lastROS2Time = 0.0;
        double m_lastRos2NodeTime = 0.0;
        float m_lastElapsed = 0.0f;
    };
} // namespace SensorDebug
