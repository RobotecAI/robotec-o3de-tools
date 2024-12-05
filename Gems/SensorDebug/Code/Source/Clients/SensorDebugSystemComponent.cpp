
#include "SensorDebugSystemComponent.h"

#include <SensorDebug/SensorDebugTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/ConsoleBus.h>
namespace SensorDebug
{
    AZ_COMPONENT_IMPL(SensorDebugSystemComponent, "SensorDebugSystemComponent", SensorDebugSystemComponentTypeId);

    void SensorDebugSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SensorDebugSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void SensorDebugSystemComponent::Activate()
    {
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void SensorDebugSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void SensorDebugSystemComponent::ClearSensors()
    {
        m_sensorEntities.clear();
        m_sensorNames.clear();
        m_entitiesNames.clear();
    }

    void SensorDebugSystemComponent::FindSensorsWithBusAPI()
    {
        ClearSensors();
        ROS2::SensorConfigurationRequestBus::EnumerateHandlers(
            [this](ROS2::SensorConfigurationRequest* handler)
            {
                // try to cast to component
                auto* component = azrtti_cast<AZ::Component*>(handler);
                if (component)
                {
                    // get entity name
                    AZ::Entity* entity = component->GetEntity();
                    AZ_Printf(
                        "TestComponent",
                        "SensorConfigurationRequestBus Handler -> %s %s",
                        entity->GetName().c_str(),
                        handler->RTTI_GetTypeName());

                    const auto pair = AZ::EntityComponentIdPair(entity->GetId(), component->GetId());
                    m_sensorEntities.push_back(pair);
                    m_sensorNames[pair] = component->RTTI_GetTypeName();
                    m_entitiesNames[pair] = entity->GetName();

                    ROS2::SensorConfiguration sensorConfig = handler->GetSensorConfiguration();
                    m_sensorFrequencies[pair] = sensorConfig.m_frequency;
                }
                return true;
            });

        AZ_Printf("TestComponent", "Found %d sensor entities", m_sensorEntities.size());
    }

    void SensorDebugSystemComponent::FindSensorsWithComponentAPI()
    {
        ClearSensors();
        auto searchFunction = [this](AZ::Entity* entity)
        {
            AZ_Assert(entity, "Entity is null");
            const auto sensorComponent = ROS2::GetSensorsForEntity(entity->GetId());
            AZ_Printf("TestComponent", "Entity  %s has %d sensors", entity->GetName().c_str(), sensorComponent.size());
            m_sensorEntities.insert(m_sensorEntities.end(), sensorComponent.begin(), sensorComponent.end());

            // collect types names
            for (const auto* component : entity->GetComponents())
            {
                if (ROS2::IsComponentROS2Sensor(component))
                {
                    const auto pair = AZ::EntityComponentIdPair(entity->GetId(), component->GetId());
                    m_sensorNames[pair] = component->RTTI_GetTypeName();
                    m_entitiesNames[pair] = entity->GetName();
                    ROS2::SensorConfiguration sensorConfig;
                    ROS2::SensorConfigurationRequestBus::EventResult(
                        sensorConfig, pair, &ROS2::SensorConfigurationRequest::GetSensorConfiguration);
                    m_sensorFrequencies[pair] = sensorConfig.m_frequency;
                }
            }
        };

        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationRequests::EnumerateEntities, searchFunction);
        AZ_Printf("TestComponent", "Found %d sensor entities", m_sensorEntities.size());
    }

    void SensorDebugSystemComponent::FindSensorsWithGivenType(const char* typeId)
    {
        ClearSensors();
        auto searchFunction = [this, typeId](AZ::Entity* entity)
        {
            AZ_Assert(entity, "Entity is null");
            const auto sensorComponent = ROS2::GetSensorsOfType(entity->GetId(), AZ::Uuid(typeId));
            AZ_Printf("TestComponent", "Entity  %s has %d sensors", entity->GetName().c_str(), sensorComponent.size());
            m_sensorEntities.insert(m_sensorEntities.end(), sensorComponent.begin(), sensorComponent.end());

            // collect types names
            for (const auto* component : entity->GetComponents())
            {
                if (ROS2::IsComponentROS2Sensor(component))
                {
                    const auto pair = AZ::EntityComponentIdPair(entity->GetId(), component->GetId());
                    m_sensorNames[pair] = component->RTTI_GetTypeName();
                    m_entitiesNames[pair] = entity->GetName();
                    ROS2::SensorConfiguration sensorConfig;
                    ROS2::SensorConfigurationRequestBus::EventResult(
                        sensorConfig, pair, &ROS2::SensorConfigurationRequest::GetSensorConfiguration);
                    m_sensorFrequencies[pair] = sensorConfig.m_frequency;
                }
            }
        };

        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationRequests::EnumerateEntities, searchFunction);
        AZ_Printf("TestComponent", "Found %d sensor entities", m_sensorEntities.size());
    }

    void SensorDebugSystemComponent::OnImGuiUpdate()
    {
        ImGui::Begin("ROS2 SensorDebugger");

        ImGui::InputFloat("Application Max FPS", &m_maxFPS);
        ImGui::SameLine();
        if (ImGui::Button("Set sys_MaxFPS"))
        {
            //disable vsync
            AZStd::string commandVsync = AZStd::string::format("vsync_interval=0");
            AzFramework::ConsoleRequestBus::Broadcast(&AzFramework::ConsoleRequests::ExecuteConsoleCommand, commandVsync.c_str());
            AZStd::string commandMaxFps = AZStd::string::format("sys_MaxFPS=%f", m_maxFPS);
            AzFramework::ConsoleRequestBus::Broadcast(&AzFramework::ConsoleRequests::ExecuteConsoleCommand, commandMaxFps.c_str());
            m_appFrequencies.clear();
        }

        float freqencySum = 0.0f;
        for (const auto& freq : m_appFrequencies)
        {
            freqencySum += freq;
        }
        const float averageFrequency = freqencySum / static_cast<float>(m_appFrequencies.size());
        // compute std deviation
        float variance = 0.0f;
        for (const auto& freq : m_appFrequencies)
        {
            variance += (freq - averageFrequency) * (freq - averageFrequency);
        }
        float stdDeviation = sqrt(variance / static_cast<float>(m_appFrequencies.size()));
        ImGui::Separator();
        ImGui::PlotHistogram("App Actual Frequency", m_appFrequencies.data(), static_cast<int>(m_appFrequencies.size()), 0);
        ImGui::Text("App Actual Frequency: %.2f Hz [ std_dev = %.2f ]", averageFrequency, stdDeviation);
        ImGui::SameLine();
        if (ImGui::Button("reset stats"))
        {
            m_appFrequencies.clear();
        }
        ImGui::Separator();
        if (ImGui::Button("Refresh with EnumerateHandlers(o3de bus API)"))
        {
            FindSensorsWithBusAPI();
        }

        if (ImGui::Button("Refresh with Helper Function"))
        {
            FindSensorsWithComponentAPI();
        }

        if (ImGui::Button("Find Cameras"))
        {
            FindSensorsWithGivenType(ROS2::ROS2CameraSensorComponentTypeId);
        }
        ImGui::SameLine();
        if (ImGui::Button("Find Contacts Sensors"))
        {
            FindSensorsWithGivenType(ROS2::ROS2ContactSensorComponentTypeId);
        }
        ImGui::SameLine();
        if (ImGui::Button("Find GNSS"))
        {
            FindSensorsWithGivenType(ROS2::ROS2GNSSSensorComponentTypeId);
        }
        ImGui::SameLine();
        if (ImGui::Button("Find IMU"))
        {
            FindSensorsWithGivenType(ROS2::ROS2ImuSensorComponentTypeId);
        }
        ImGui::SameLine();
        if (ImGui::Button("Find Lidar2D"))
        {
            FindSensorsWithGivenType(ROS2::ROS2Lidar2DSensorComponentTypeId);
        }
        ImGui::SameLine();
        if (ImGui::Button("Find Lidar"))
        {
            FindSensorsWithGivenType(ROS2::ROS2LidarSensorComponentTypeId);
        }
        ImGui::SameLine();
        if (ImGui::Button("Find WheelOdometer"))
        {
            FindSensorsWithGivenType(ROS2::ROS2WheelOdometryComponentTypeId);
        }
        ImGui::SameLine();
        if (ImGui::Button("Find Odometry"))
        {
            FindSensorsWithGivenType(ROS2::ROS2OdometrySensorComponent);
        }
        ImGui::InputInt("History Size", &m_historySize);
        for (auto& sensorEntity : m_sensorEntities)
        {
            ImGui::Separator();
            // get name of the sensor type
            AZStd::string cookie = AZStd::string::format("##%llu%llu", AZ::u64(sensorEntity.GetEntityId()), sensorEntity.GetComponentId());
            const auto& sensorName = m_sensorNames[sensorEntity];
            const auto& entityName = m_entitiesNames[sensorEntity];
            float frequency = 0.0f;
            ROS2::SensorConfigurationRequestBus::EventResult(
                frequency, sensorEntity, &ROS2::SensorConfigurationRequest::GetEffectiveFrequency);

            m_sensorFrequencyHistory[sensorEntity].push_back(frequency);
            auto &sensorFrequencyHistory = m_sensorFrequencyHistory[sensorEntity];
            if (sensorFrequencyHistory.size() > m_historySize)
            {
                sensorFrequencyHistory.erase(sensorFrequencyHistory.begin());
            }
            float freqencySum = 0.0f;
            for (const auto& freq : sensorFrequencyHistory)
            {
                    freqencySum += freq;
            }
            const float averageFrequency = freqencySum / static_cast<float>(sensorFrequencyHistory.size());
            // compute std deviation
            float variance = 0.0f;
            for (const auto& freq : sensorFrequencyHistory)
            {
                    variance += (freq - averageFrequency) * (freq - averageFrequency);
            }
            float stdDeviation = sqrt(variance / static_cast<float>(sensorFrequencyHistory.size()));
            const AZStd::string histogramNameWithCookie = AZStd::string::format("Histogram%s", cookie.c_str());

            ImGui::PlotHistogram(
                histogramNameWithCookie.c_str(), sensorFrequencyHistory.data(), static_cast<int>(sensorFrequencyHistory.size()), 0);
            ImGui::SameLine();
            const AZStd::string resetButton = AZStd::string::format("reset stats%s", cookie.c_str());
            if (ImGui::Button(resetButton.c_str()))
            {
                    sensorFrequencyHistory.clear();
            }
            ImGui::Text(
                "%s : %s effective Freq: %.2f Hz [ std_dev = %.2f ]",
                entityName.c_str(),
                sensorName.c_str(),
                averageFrequency,
                stdDeviation);

            AZStd::string buttonNameEna = AZStd::string::format("Enable%s", cookie.c_str());
            AZStd::string buttonNameDis = AZStd::string::format("Disable%s", cookie.c_str());

            if (ImGui::Button(buttonNameEna.c_str()))
            {
                ROS2::SensorConfigurationRequestBus::Event(sensorEntity, &ROS2::SensorConfigurationRequest::SetSensorEnabled, true);
            }
            ImGui::SameLine();
            if (ImGui::Button(buttonNameDis.c_str()))
            {
                ROS2::SensorConfigurationRequestBus::Event(sensorEntity, &ROS2::SensorConfigurationRequest::SetSensorEnabled, false);
            }

            AZStd::string buttonNamePubEna = AZStd::string::format("Enable Publication%s", cookie.c_str());
            AZStd::string buttonNamePubDis = AZStd::string::format("Disable Publication%s", cookie.c_str());

            if (ImGui::Button(buttonNamePubEna.c_str()))
            {
                ROS2::SensorConfigurationRequestBus::Event(sensorEntity, &ROS2::SensorConfigurationRequest::SetPublishingEnabled, true);
            }
            ImGui::SameLine();
            if (ImGui::Button(buttonNamePubDis.c_str()))
            {
                ROS2::SensorConfigurationRequestBus::Event(sensorEntity, &ROS2::SensorConfigurationRequest::SetPublishingEnabled, false);
            }

            AZStd::string buttonNameVisEna = AZStd::string::format("Enable Vis%s", cookie.c_str());
            AZStd::string buttonNameVisDis = AZStd::string::format("Disable Vis%s", cookie.c_str());

            if (ImGui::Button(buttonNameVisEna.c_str()))
            {
                ROS2::SensorConfigurationRequestBus::Event(sensorEntity, &ROS2::SensorConfigurationRequest::SetVisualizeEnabled, true);
            }
            ImGui::SameLine();
            if (ImGui::Button(buttonNameVisDis.c_str()))
            {
                ROS2::SensorConfigurationRequestBus::Event(sensorEntity, &ROS2::SensorConfigurationRequest::SetVisualizeEnabled, false);
            }
            AZStd::string freqName = AZStd::string::format("Frequency%s", cookie.c_str());
            ImGui::InputFloat(freqName.c_str(), &m_sensorFrequencies[sensorEntity]);
            ImGui::SameLine();
            AZStd::string buttonSetFreq = AZStd::string::format("Set Frequency%s", cookie.c_str());
            if (ImGui::Button(buttonSetFreq.c_str()))
            {
                ROS2::SensorConfigurationRequestBus::Event(
                    sensorEntity, &ROS2::SensorConfigurationRequest::SetDesiredFrequency, m_sensorFrequencies[sensorEntity]);
            }
        }

        ImGui::End();
    }

    void SensorDebugSystemComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        m_appFrequencies.push_back(1.0f / deltaTime);
        if (m_appFrequencies.size() > m_historySize)
        {
            m_appFrequencies.erase(m_appFrequencies.begin());
        }
    }
} // namespace SensorDebug
