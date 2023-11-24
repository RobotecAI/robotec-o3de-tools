
#pragma once

namespace RobotecRecordingTools
{
    // System Component TypeIds
    inline constexpr const char* RobotecRecordingToolsSystemComponentTypeId = "{4DF735D7-F5B2-4F34-B7B1-CC2861D84C1F}";
    inline constexpr const char* RobotecRecordingToolsEditorSystemComponentTypeId = "{C8E9AE7C-ABAA-49B4-A044-8267E4F3AB2C}";

    // Module derived classes TypeIds
    inline constexpr const char* RobotecRecordingToolsModuleInterfaceTypeId = "{2540E113-4D8B-4C80-BA7E-DB5D38482E69}";
    inline constexpr const char* RobotecRecordingToolsModuleTypeId = "{3A9A1640-A842-47A4-92C0-B716D40D3808}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* RobotecRecordingToolsEditorModuleTypeId = RobotecRecordingToolsModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* RobotecRecordingToolsRequestsTypeId = "{03EDB813-C987-417C-9DB7-42BAB8337BE8}";
} // namespace RobotecRecordingTools
