
#pragma once

namespace RobotecSpectatorCamera
{
    // System Component TypeIds
    inline constexpr const char* RobotecSpectatorCameraSystemComponentTypeId = "{5D49C900-EFD1-46AD-A7B2-B1736A3FC6DB}";
    inline constexpr const char* RobotecSpectatorCameraEditorSystemComponentTypeId = "{2D523636-EB9D-4136-ACF0-F61518100E76}";

    // Module derived classes TypeIds
    inline constexpr const char* RobotecSpectatorCameraModuleInterfaceTypeId = "{4DA45FE1-73FA-413E-9D76-FEC0DD427679}";
    inline constexpr const char* RobotecSpectatorCameraModuleTypeId = "{9FB4D1FC-BE7D-4B3F-8DFA-7441F156AC58}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* RobotecSpectatorCameraEditorModuleTypeId = RobotecSpectatorCameraModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* RobotecSpectatorCameraRequestsTypeId = "{71A98554-5F3A-4B9E-BDCB-A6F3213D8D2C}";

    // Component TypeIds
    inline constexpr const char* SpectatorCameraEditorComponentTypeId = "{4bff4f83-42a1-4b4e-99b4-ce5fd33eec7f}";
    inline constexpr const char* SpectatorCameraComponentTypeId = "{e1a76b62-3557-4495-959d-7e4779323c73}";

    // Configuration TypeIds
    inline constexpr const char* SpectatorCameraConfigurationTypeId = "{e5344b04-85ab-44b2-b90a-bf271c33149e}";
} // namespace RobotecSpectatorCamera
