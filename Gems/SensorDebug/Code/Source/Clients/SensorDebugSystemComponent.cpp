
#include "SensorDebugSystemComponent.h"

#include <SensorDebug/SensorDebugTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/ConsoleBus.h>

#include <AzCore/Time/ITime.h>
#include <PhysX/Configuration/PhysXConfiguration.h>

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
        GetPhysXConfig();
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

    AzPhysics::Scene* SensorDebugSystemComponent::GetScene()
    {
        if (m_scene)
        {
            return m_scene;
        }
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system.");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface.");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        if (defaultSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return nullptr;
        }
        AzPhysics::Scene* scene = sceneInterface->GetScene(defaultSceneHandle);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene pointer.");
        m_scene = scene;
        return scene;
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

    void SensorDebugSystemComponent::Pause(bool isPaused)
    {
        GetScene()->SetEnabled(!isPaused);
    }

    void SensorDebugSystemComponent::UpdatePhysXConfig()
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system.");
        auto* physicsSystemConfigurationPtr = physicsSystem->GetConfiguration();
        auto* physicsSystemConfiguration = azdynamic_cast<const PhysX::PhysXSystemConfiguration*>(physicsSystemConfigurationPtr);
        AZ_Assert(physicsSystemConfiguration, "Invalid physics system configuration pointer, a new Physics system in O3DE????");
        physicsSystem->UpdateConfiguration(&m_modifiedPhysXConfig, true);
    }

    void SensorDebugSystemComponent::GetPhysXConfig()
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system.");
        auto* physicsSystemConfigurationPtr = physicsSystem->GetConfiguration();
        auto* physicsSystemConfiguration = azdynamic_cast<const PhysX::PhysXSystemConfiguration*>(physicsSystemConfigurationPtr);
        AZ_Assert(physicsSystemConfiguration, "Invalid physics system configuration pointer, a new Physics system in O3DE????");
        m_modifiedPhysXConfig = *physicsSystemConfiguration;
    }

    AZStd::string GetSecondAnimation(double value)
    {
        double part = value - AZStd::floor(value);
        size_t numStars = static_cast<size_t>(part * 60);

        AZStd::string anim (61, ' ');
        numStars = AZStd::min(numStars, anim.length());
        for (size_t i = 0; i <numStars; i++)
        {
            anim[i] = '*';
        }
        return anim;
    }
    void SensorDebugSystemComponent::OnImGuiUpdate()
    {
        ImGui::Begin("ROS2 SensorDebugger");
        ImGui::Separator();
        ImGui::Text("TimeyWimey Stuff");
        const auto ros2Intreface = ROS2::ROS2Interface::Get();
        auto ros2ts = ros2Intreface ? ROS2::ROS2Interface::Get()->GetROSTimestamp() : builtin_interfaces::msg::Time();
        const float ros2tsSec = ros2ts.sec + ros2ts.nanosec / 1e9;
        auto ros2Node = ros2Intreface ? ROS2::ROS2Interface::Get()->GetNode() : nullptr;
        auto nodeTime = ros2Node ? ros2Node->now() : rclcpp::Time(0, 0);
        const double ros2nodetsSec = nodeTime.seconds();

        auto timeSystem = AZ::Interface<AZ::ITime>::Get();
        const auto elapsedTime = timeSystem ? static_cast<double>(timeSystem->GetElapsedTimeUs()) / 1e6 : 0.0;

        ImGui::Text("Current ROS 2 time (Gem)  : %f %s", ros2tsSec, GetSecondAnimation(ros2tsSec).c_str());
        ImGui::Text("Current ROS 2 time (Node) : %f %s", ros2nodetsSec, GetSecondAnimation(ros2nodetsSec).c_str());
        ImGui::Text("Current O3DE time         : %f %s", elapsedTime, GetSecondAnimation(elapsedTime).c_str());

        ImGui::Text("Tick ROS 2 time (Gem)     : %f", ros2tsSec-m_lastROS2Time);
        ImGui::Text("Tick ROS 2 time (Node)    : %f", ros2nodetsSec-m_lastRos2NodeTime);
        ImGui::Text("Tick O3DE time            : %f", elapsedTime-m_lastElapsed);



        m_lastElapsed = elapsedTime;
        m_lastROS2Time = ros2tsSec;
        m_lastRos2NodeTime = ros2nodetsSec;
        ImGui::Separator();
        ImGui::Text("PhysX");
        ImGui::InputFloat("Fixed timestamp", &m_modifiedPhysXConfig.m_fixedTimestep, 0.0f, 0.0f, "%.6f");
        ImGui::InputFloat("Max timestamp", &m_modifiedPhysXConfig.m_maxTimestep, 0.0f, 0.0f, "%.6f");
        ImGui::InputFloat("RealTime factor", &m_modifiedPhysXConfig.m_realTimeFactor, 0.0f, 0.0f, "%.1f");
        if (ImGui::Button("Update PhysX Config"))
        {
            UpdatePhysXConfig();
        }
        ImGui::SameLine();
        if (ImGui::Button("Pause"))
        {
            Pause(true);
        }
        ImGui::SameLine();
        if (ImGui::Button("Unpause"))
        {
            Pause(false);
        }
        ImGui::Separator();
        ImGui::Text("Atom");

        ImGui::InputFloat("Application Max FPS", &m_maxFPS);
        ImGui::SameLine();
        if (ImGui::Button("Set sys_MaxFPS"))
        {
            // disable vsync
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
            auto& sensorFrequencyHistory = m_sensorFrequencyHistory[sensorEntity];
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
