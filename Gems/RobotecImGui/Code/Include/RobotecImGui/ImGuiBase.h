/*
 * Copyright (c) Robotec.ai 2023. All rights reserved.
 */

#include "ImGuiUtilityBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <ImGuiBus.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <imgui/imgui.h>

#pragma once

namespace RobotecImGui
{
    //! Class ImGuiBase serves as a foundational component for integrating ImGui.
    //! This class inherits from ImGuiUtilityRequestBus::Handler (custom interface for inheriting classes to implement specific behavior)
    //! and ImGui::ImGuiUpdateListenerBus::Handler (for handling ImGui frame updates). Key Features:
    //! - GetMessage: Virtual function to be implemented by derived classes for fetching ImGui elements.
    //! - OnImGuiUpdate: Handles ImGui frame updates, drawing elements on the ImGui window.
    //! - OnImGuiMainMenuUpdate: Integrates custom menu items into the ImGui main menu.
    //! Usage: Extend this class to create custom ImGui interfaces for different component types, particularly for sensor data visualization
    //! in a ROS2 environment.

    class ImGuiBase
        : public ImGuiUtilityRequestBus::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , public AZ::Component
    {
    public:
        AZStd::vector<ImGuiElement> GetMessage() = 0;

        AZStd::string GetEntityName() override
        {
            AZ::Entity* entity = GetEntity();
            if (!entity)
            {
                return "";
            }
            return entity->GetName();
        }

        AZStd::vector<AZ::Component*> GetEntitySensorComponents(AZ::Entity* entity)
        {
            if (!entity)
            {
                AZ_Error("ImGuiBase", false, "Invalid entity provided to Draw method.");
                return {};
            }

            auto childComponents = GetEntityComponents(entity->GetId());
            if (childComponents.empty())
            {
                AZ_Warning("ImGuiBase", false, "No child components found for the entity.");
                return {};
            }
            FilterChildComponents(childComponents);
            return childComponents;
        }

    private:
        bool m_showRos2ImGui{ false };
        AZStd::vector<ImGuiElement> imGuiElements;

    protected:
        void OnImGuiUpdate() override
        {
            if (m_showRos2ImGui)
            {
                imGuiElements.clear();
                AZStd::vector<ImGuiElement> elements;
                ImGuiUtilityRequestBus::EventResult(elements, GetEntityId(), &ImGuiRequests::GetMessage);
                imGuiElements.insert(imGuiElements.end(), elements.begin(), elements.end());

                for (auto& element : imGuiElements)
                {
                    element.Draw();
                }
            }
            imGuiElements.clear();
        }

        void OnImGuiMainMenuUpdate() override
        {
            if (ImGui::BeginMenu("ROS 2 ImGui"))
            {
                AZStd::string entityName;
                ImGuiUtilityRequestBus::EventResult(entityName, GetEntityId(), &ImGuiRequests::GetEntityName);
                if (ImGui::MenuItem(entityName.c_str(), "", &m_showRos2ImGui))
                {
                }
                ImGui::EndMenu();
            }
        }

        void FilterChildComponents(AZStd::vector<AZ::Component*>& components)
        {
            components.erase(
                std::remove_if(
                    components.begin(),
                    components.end(),
                    [this](AZ::Component* component)
                    {
                        return !IsSensorComponent(component) || IsImGuiComponent(component);
                    }),
                components.end());
        }

        AZStd::vector<AZ::Component*> GetEntityComponents(AZ::EntityId entityId)
        {
            AZStd::vector<AZ::Component*> components;
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            if (entity)
            {
                components = entity->GetComponents();
            }
            else
            {
                AZ_Error("ImGuiBase", false, "Could not find entity with ID: %llu", static_cast<unsigned long long>(entityId));
            }
            return components;
        }

        bool EndsWith(const std::string& value, const std::string& ending) const
        {
            if (ending.size() > value.size())
                return false;
            return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
        }

        bool IsSensorComponent(const AZ::Component* component) const
        {
            std::string typeName = component->RTTI_GetTypeName();
            return EndsWith(typeName, "SensorComponent");
        }

        bool IsImGuiComponent(const AZ::Component* component) const
        {
            std::string typeName = component->RTTI_GetTypeName();
            return EndsWith(typeName, "ImGuiComponent");
        }
    };

} // namespace RobotecImGui
