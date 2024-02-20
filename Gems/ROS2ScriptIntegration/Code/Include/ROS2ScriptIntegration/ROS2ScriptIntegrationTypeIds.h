
#pragma once

namespace ROS2ScriptIntegration
{
    // System Component TypeIds
    inline constexpr const char* ROS2ScriptIntegrationSystemComponentTypeId = "{9F27705B-7F94-47D6-BC9D-67C1541A68E2}";
    inline constexpr const char* ROS2ScriptIntegrationEditorSystemComponentTypeId = "{60221622-30B3-4561-A082-9E306D2D252A}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2ScriptIntegrationModuleInterfaceTypeId = "{30EAAB05-7D98-4B7D-A461-8E5EEAF24F6A}";
    inline constexpr const char* ROS2ScriptIntegrationModuleTypeId = "{2BE7A2BA-862D-42F0-B4F4-8C4746C08632}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2ScriptIntegrationEditorModuleTypeId = ROS2ScriptIntegrationModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2ScriptIntegrationRequestsTypeId = "{BF116726-F2D1-44A2-ADE9-FB0047E6749D}";
} // namespace ROS2ScriptIntegration
