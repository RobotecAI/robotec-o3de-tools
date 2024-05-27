
#pragma once

namespace Pointcloud
{
    // System Component TypeIds
    inline constexpr const char* PointcloudSystemComponentTypeId = "{4D06BD24-03C2-400B-859C-4BC0A17D03AE}";
    inline constexpr const char* PointcloudEditorSystemComponentTypeId = "{840D5CF4-4598-44C8-AFD0-94D5F7D7B441}";

    // Module derived classes TypeIds
    inline constexpr const char* PointcloudModuleInterfaceTypeId = "{348249A1-6434-4F67-A19B-696CF7122A98}";
    inline constexpr const char* PointcloudModuleTypeId = "{0C8CF2FF-AAE1-4875-8A4F-BF6EB7723140}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* PointcloudEditorModuleTypeId = PointcloudModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* PointcloudRequestsTypeId = "{19964F12-8F4F-4CA3-955C-A7602FFDD477}";
} // namespace Pointcloud
