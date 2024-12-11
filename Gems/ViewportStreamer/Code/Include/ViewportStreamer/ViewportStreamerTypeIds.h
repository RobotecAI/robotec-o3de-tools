
#pragma once

namespace ViewportStreamer
{
    // System Component TypeIds
    inline constexpr const char* ViewportStreamerComponentTypeId = "{1EF0EC12-33C2-45B7-B637-CAAF8DF7C25A}";

    // Module derived classes TypeIds
    inline constexpr const char* ViewportStreamerModuleInterfaceTypeId = "{5C5BB679-067D-450E-8A11-A5E52DFF9B34}";
    inline constexpr const char* ViewportStreamerModuleTypeId = "{15FBBD78-71BC-4D43-B8E5-51AEC20DBE67}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ViewportStreamerEditorModuleTypeId = ViewportStreamerModuleTypeId;
} // namespace ViewportStreamer
