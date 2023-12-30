/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotecImGui/ImGuiBase.h>

#pragma once

namespace RobotecImGui
{
    class SensorImGui
        : public RobotecImGui::ImGuiBase
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
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

    private:
        AZStd::vector<ImGuiElement> m_imGuiElements;
        AZ::Entity* entity;
                
    };

} // namespace RobotecImGui
