
#pragma once

namespace BillboardComponent
{
    // System Component TypeIds
    inline constexpr const char* BillboardSystemComponentTypeId = "{ACC1D1A3-A0F9-4620-AB9C-742D879E1951}";
    inline constexpr const char* BillboardEditorSystemComponentTypeId = "{F54E7F75-6826-46DC-A2EC-C981BCBD3D92}";

    // Module derived classes TypeIds
    inline constexpr const char* BillboardModuleInterfaceTypeId = "{7F255600-7C6D-437C-B5CC-8BE9882545DD}";
    inline constexpr const char* BillboardModuleTypeId = "{40012893-27E8-4F53-8FF9-34DD51E0050D}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* BillboardEditorModuleTypeId = BillboardModuleTypeId;

    // Component TypeIds
    inline constexpr const char* BillboardComponentTypeId = "{ad8ebcee-5d2b-486b-9d43-06d4c0369812}";
} // namespace BillboardComponent
