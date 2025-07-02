
#pragma once

namespace RandomizeUtils
{

    // Module derived classes TypeIds
    inline constexpr const char* RandomizeUtilsModuleInterfaceTypeId = "{0C8DC567-4080-4647-9AFF-FD03E6593A27}";
    inline constexpr const char* RandomizeUtilsModuleTypeId = "{8D073235-A1A1-4EE6-9736-40E640F59BC7}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* RandomizeUtilsEditorModuleTypeId = RandomizeUtilsModuleTypeId;

    inline constexpr const char* RandomizePoseComponentTypeId = "{4e990b5f-062b-4aaa-8e2a-440d3574ff1e}";
} // namespace RandomizeUtils
