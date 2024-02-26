
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

    inline constexpr const char* PublisherSystemComponentTypeId = "{6ee12dfa-5a6c-4223-b067-f798d9127840}";
    inline constexpr const char* SubscriberSystemComponentTypeId = "{a30c445b-9a51-4036-871d-9f3b2917de58}";
    inline constexpr const char* PublisherEditorSystemComponentTypeId = "{e31ac401-f40e-442a-8072-19d6c1f523f2}";
    inline constexpr const char* SubscriberEditorSystemComponentTypeId = "{f4dba5a4-23da-463a-8d20-5866e897366f}";

    // Interface TypeIds
    inline constexpr const char* ROS2ScriptIntegrationRequestsTypeId = "{BF116726-F2D1-44A2-ADE9-FB0047E6749D}";

} // namespace ROS2ScriptIntegration
