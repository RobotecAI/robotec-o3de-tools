
#pragma once

namespace ExposeConsoleToRos
{
    // System Component TypeIds
    inline constexpr const char* ExposeConsoleToRosSystemComponentTypeId = "{7D8DB10F-7B2C-4E91-9922-F1EBD3F1430A}";
    inline constexpr const char* ExposeConsoleToRosEditorSystemComponentTypeId = "{27D3FF8B-B6BF-4865-B238-8EE292911C6C}";

    // Module derived classes TypeIds
    inline constexpr const char* ExposeConsoleToRosModuleInterfaceTypeId = "{209AEF62-0F20-48F0-88F0-89CDE002900E}";
    inline constexpr const char* ExposeConsoleToRosModuleTypeId = "{147C9841-688C-4DE3-BE64-765F7B4D5219}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ExposeConsoleToRosEditorModuleTypeId = ExposeConsoleToRosModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ExposeConsoleToRosRequestsTypeId = "{BF7FDA4A-BF1E-4DFD-9F11-8BE4AC3834A4}";
} // namespace ExposeConsoleToRos
