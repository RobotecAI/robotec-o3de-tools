
#pragma once

namespace RobotecImGui
{
    // System Component TypeIds
    inline constexpr const char* RobotecImGuiSystemComponentTypeId = "{0EAA4B28-CDB1-4A4A-8B75-1BFDB1ABB07E}";
    inline constexpr const char* RobotecImGuiEditorSystemComponentTypeId = "{B1E3DF96-FD42-49C2-9458-28F3A8C59728}";

    // Module derived classes TypeIds
    inline constexpr const char* RobotecImGuiModuleInterfaceTypeId = "{158A7043-87AC-448E-A640-EE8A14F0CABA}";
    inline constexpr const char* RobotecImGuiModuleTypeId = "{DEE8E6AD-104E-4983-AB9A-40FF33E28E7A}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* RobotecImGuiEditorModuleTypeId = RobotecImGuiModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* RobotecImGuiRequestsTypeId = "{E3C49C3C-A447-46A7-96B7-D25900B1A587}";
} // namespace RobotecImGui
