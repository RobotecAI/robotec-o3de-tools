
#pragma once

namespace ROS2ScriptIntegration
{
    // Module derived classes TypeIds
    inline constexpr const char* ROS2ScriptIntegrationModuleInterfaceTypeId = "{30EAAB05-7D98-4B7D-A461-8E5EEAF24F6A}";
    inline constexpr const char* ROS2ScriptIntegrationModuleTypeId = "{2BE7A2BA-862D-42F0-B4F4-8C4746C08632}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2ScriptIntegrationEditorModuleTypeId = ROS2ScriptIntegrationModuleTypeId;

    inline constexpr const char* PublisherSystemComponentTypeId = "{6EE12DFA-5A6C-4223-B067-F798D9127840}";
    inline constexpr const char* SubscriberSystemComponentTypeId = "{A30C445B-9A51-4036-871D-9F3B2917DE58}";
    inline constexpr const char* PublisherEditorSystemComponentTypeId = "{E31AC401-F40E-442A-8072-19D6C1F523F2}";
    inline constexpr const char* SubscriberEditorSystemComponentTypeId = "{F4DBA5A4-23DA-463A-8D20-5866E897366F}";

    // Interface TypeIds
    inline constexpr const char* ROS2ScriptPublisherRequestsTypeId = "{B8356874-F7BA-4436-8D98-A342D7C720D9}";
    inline constexpr const char* ROS2ScriptSubscriberRequestsTypeId = "{71935101-17DE-4636-97F8-DEA68938706D}";
    inline constexpr const char* ROS2ScriptSubscriberNotificationsTypeId = "{A64EBBC0-3C6E-44F5-8A58-EA921AFA1C15}";

} // namespace ROS2ScriptIntegration
