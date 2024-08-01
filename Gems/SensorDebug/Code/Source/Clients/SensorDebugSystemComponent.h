
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

#include <ImGuiBus.h>
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
        void FindSensorsWithBusAPI();
        void FindSensorsWithComponentAPI();
        void FindSensorsWithGivenType(const char* typeId);
        AZStd::vector<AZ::EntityComponentIdPair> m_sensorEntities;
        AZStd::unordered_map<AZ::EntityComponentIdPair, AZStd::string> m_sensorNames;
        AZStd::unordered_map<AZ::EntityComponentIdPair, AZStd::string> m_entitiesNames;
        AZStd::unordered_map<AZ::EntityComponentIdPair, float> m_sensorFrequencies;
    };
} // namespace SensorDebug
