
#pragma once

namespace ROS2PoseControl
{
    // System Component TypeIds
    inline constexpr const char* ROS2PoseControlSystemComponentTypeId = "{02BD37CB-AD82-4BBB-A420-9C988CF96435}";
    inline constexpr const char* ROS2PoseControlEditorSystemComponentTypeId = "{4F6703CB-248F-4633-9455-662FEC5C22A1}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2PoseControlModuleInterfaceTypeId = "{914368C1-9338-43A1-8D8A-4053BAA0ABB6}";
    inline constexpr const char* ROS2PoseControlModuleTypeId = "{A2F9C561-BBD7-4C05-9C1B-EB879B3F56B7}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2PoseControlEditorModuleTypeId = ROS2PoseControlModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2PoseControlRequestsTypeId = "{CAAF2852-6CCF-4EDE-9C8D-308FF63117F1}";
} // namespace ROS2PoseControl
