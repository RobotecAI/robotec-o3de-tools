/*
 * Copyright (c) Robotec.ai 2023. All rights reserved.
 */

#include <RobotecImGui/ImGuiBase.h>

#pragma once

namespace RobotecImGui
{
    //! Class responsible for drawing ImGui elements for ROS 2 sensors
    //! Inherits from ImGuiBase and implements GetMessage method
    //! Usage: Add this component to any entity with ROS 2 sensor components and it will draw ImGui elements for them
    //! Functionality: disable/enable publishing from the sensor

    class SensorImGui : public RobotecImGui::ImGuiBase
    {
    public:
        SensorImGui();
        ~SensorImGui();

        AZ_COMPONENT(SensorImGui, "{76273BFF-4673-40CA-8823-1116BD1EF284}", AZ::Component);

        AZStd::vector<ImGuiElement> GetMessage() override;

        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

    private:
        AZStd::vector<ImGuiElement> m_imGuiElements;
        AZ::Entity* entity;
    };

} // namespace RobotecImGui
