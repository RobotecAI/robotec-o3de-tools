
#pragma once

namespace RandomizeUtils
{
    // System Component TypeIds
    inline constexpr const char* RandomizeUtilsSystemComponentTypeId = "{699FF1A1-D445-4EC4-815C-D092A2579F2E}";
    inline constexpr const char* RandomizeUtilsEditorSystemComponentTypeId = "{02979C13-3598-4904-9A95-7FFA4264C863}";

    // Module derived classes TypeIds
    inline constexpr const char* RandomizeUtilsModuleInterfaceTypeId = "{0C8DC567-4080-4647-9AFF-FD03E6593A27}";
    inline constexpr const char* RandomizeUtilsModuleTypeId = "{8D073235-A1A1-4EE6-9736-40E640F59BC7}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* RandomizeUtilsEditorModuleTypeId = RandomizeUtilsModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* RandomizeUtilsRequestsTypeId = "{A75134B2-B94D-47FC-B03D-36DE08B97269}";
} // namespace RandomizeUtils
