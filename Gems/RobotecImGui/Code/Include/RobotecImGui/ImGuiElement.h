/*
 * Copyright (c) Robotec.ai 2023. All rights reserved.
 */

#pragma once

#include <imgui/imgui.h>
#include <AzCore/std/string/string.h>

namespace RobotecImGui
{

    struct ImGuiElement
    {
        AZStd::string label; // Used for text
        AZStd::string buttonText; // Used for button
        bool hasButton; // Flag to indicate if a button should be drawn
        bool state; // For Checkbox
        bool hasCheckbox; // Flag to indicate if a checkbox should be drawn
        std::function<void()> buttonCallback; // Callback for button click
        std::function<void(bool)> checkboxCallback; // Callback for checkbox change
        bool prevState;

        ImGuiElement(
            const AZStd::string& label = "",
            const AZStd::string& buttonText = "",
            bool hasButton = false,
            bool hasCheckbox = false,
            std::function<void()> buttonCallback = nullptr,
            std::function<void(bool)> checkboxCallback = nullptr)
            : label(label)
            , buttonText(buttonText)
            , hasButton(hasButton)
            , hasCheckbox(hasCheckbox)
            , state(false)
            , buttonCallback(std::move(buttonCallback))
            , checkboxCallback(std::move(checkboxCallback))
        {
        }

        void Draw()
        {
            if (!label.empty())
            {
                ImGui::Text("%s", label.c_str());
            }

            if (hasCheckbox)
            {
                prevState = state;
                ImGui::PushID(this); // Checkboxes need unique IDs
                ImGui::Checkbox("##Checkbox", &state);
                ImGui::SameLine();
                if (state != prevState && checkboxCallback)
                {
                    checkboxCallback(state);
                }
                ImGui::PopID();
            }

            if (hasButton && !buttonText.empty())
            {
                ImGui::PushID(this); // Buttons need unique IDs
                if (ImGui::Button(buttonText.c_str()) && buttonCallback)
                {
                    buttonCallback();
                }
                ImGui::PopID();
            }
        }
    };

} // namespace RobotecImGui
