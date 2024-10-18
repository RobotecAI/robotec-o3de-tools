
#pragma once

namespace PhysicalTimeTools
{
    // System Component TypeIds
    inline constexpr const char* PhysicalTimeToolsSystemComponentTypeId = "{41CB3647-F9C1-4586-A444-C5F0FECC6937}";
    inline constexpr const char* PhysicalTimeToolsEditorSystemComponentTypeId = "{FAC3758F-9941-4D56-87EB-62EAD7F8928C}";

    // Module derived classes TypeIds
    inline constexpr const char* PhysicalTimeToolsModuleInterfaceTypeId = "{4910B103-FDC9-4731-9789-9C61B952686B}";
    inline constexpr const char* PhysicalTimeToolsModuleTypeId = "{AB905F97-F2B7-4D9B-80FA-F0A8BE084ACA}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* PhysicalTimeToolsEditorModuleTypeId = PhysicalTimeToolsModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* PhysicalTimeToolsRequestsTypeId = "{11128F0D-4FAB-4DF1-A354-1B1BF4A6D9EB}";
} // namespace PhysicalTimeTools
