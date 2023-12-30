/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SensorImGui.h"

#include <AzCore/Component/Entity.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2/Sensor/SensorConfiguration.h>

#include <algorithm>

namespace RobotecImGui
{
    AZStd::vector<ImGuiElement> SensorImGui::GetMessage()
    {
        m_imGuiElements.clear();
        const auto childComponents = GetEntitySensorComponents(entity);
        if (childComponents.empty())
        {
            return {};
        }

        for (const auto& component : childComponents)
        {
            std::shared_ptr<ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>> sharedSensorComponentTickBased;
            std::shared_ptr<ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>> sharedSensorComponentPhysicsBased;

            if (auto* tickBased = azrtti_cast<ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>*>(component))
            {
                sharedSensorComponentTickBased = std::shared_ptr<ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>>(
                    tickBased,
                    [](auto*)
                    {
                    });
            }
            else if (auto* physicsBased = azrtti_cast<ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>*>(component))
            {
                sharedSensorComponentPhysicsBased = std::shared_ptr<ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>>(
                    physicsBased,
                    [](auto*)
                    {
                    });
            }

            if (!sharedSensorComponentTickBased && !sharedSensorComponentPhysicsBased)
            {
                AZ_Error("SensorImGui", false, "Invalid sensor component type.");
                continue; // Skip to the next component
            }

            // Common logic for both types of components
            ROS2::SensorConfiguration sensorConfiguration = sharedSensorComponentTickBased
                ? sharedSensorComponentTickBased->GetSensorConfiguration()
                : sharedSensorComponentPhysicsBased->GetSensorConfiguration();

            AZStd::string label;
            for (const auto& [key, value] : sensorConfiguration.m_publishersConfigurations)
            {
                label += "Topic: " + value.m_topic + "\n";
            }

            AZStd::string buttonText = sensorConfiguration.m_publishingEnabled ? "Disable Publishing" : "Enable Publishing";

            auto buttonCallback = [sharedSensorComponentTickBased, sharedSensorComponentPhysicsBased, sensorConfiguration]() mutable
            {
                sensorConfiguration.m_publishingEnabled = !sensorConfiguration.m_publishingEnabled;
                if (sharedSensorComponentTickBased)
                {
                    sharedSensorComponentTickBased->SetSensorConfiguration(sensorConfiguration);
                }
                else if (sharedSensorComponentPhysicsBased)
                {
                    sharedSensorComponentPhysicsBased->SetSensorConfiguration(sensorConfiguration);
                }
            };

            m_imGuiElements.emplace_back(label, buttonText, true /* hasButton */, false /* hasCheckbox */, buttonCallback);
        }
        return m_imGuiElements;
    }

    void SensorImGui::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SensorImGui, AZ::Component>()->Version(0);
            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SensorImGui>("Robotec Sensor ImGui", "ImGui for ROS 2 Sensors")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ImGui")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
            }
        }
    }

    SensorImGui::SensorImGui()
    {
    }

    SensorImGui::~SensorImGui()
    {
    }

    void SensorImGui::Activate()
    {
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        RobotecImGui::ImGuiUtilityRequestBus::Handler::BusConnect(GetEntityId());
        entity = GetEntity();
        m_imGuiElements.clear();
    }

    void SensorImGui::Deactivate()
    {
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        RobotecImGui::ImGuiUtilityRequestBus::Handler::BusDisconnect();
    }

    void SensorImGui::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RobotecSensorImGuiComponent"));
    }

    void SensorImGui::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RobotecSensorImGuiComponent"));
    }

    void SensorImGui::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void SensorImGui::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

} // namespace RobotecImGui
