
#pragma once

namespace SplineTools
{
    // System Component TypeIds
    inline constexpr const char* SplineToolsSystemComponentTypeId = "{DA4E719D-788E-4EDE-B15F-A573D1593654}";
    inline constexpr const char* SplineToolsEditorSystemComponentTypeId = "{47C73609-D74D-417D-BC1F-6A16C746C5FF}";

    // Module derived classes TypeIds
    inline constexpr const char* SplineToolsModuleInterfaceTypeId = "{26BC6C98-0D16-4594-ADA0-93FEA9EC6B19}";
    inline constexpr const char* SplineToolsModuleTypeId = "{EC2E028E-34B2-45A4-B723-04C768E3222D}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* SplineToolsEditorModuleTypeId = SplineToolsModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* SplineToolsRequestsTypeId = "{8D969183-259F-4790-9C5C-4F25BD2746FD}";
} // namespace SplineTools
