
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

    inline constexpr const char* VisualizeSplineComponentTypeId = "{5D69075B-BB8A-4336-829A-B3A84FB6DCE8}";

    inline constexpr const char* SplineSubscriberComponentTypeId = "{89B8A92A-8F17-4C30-AE0D-6B088C133283}";
    inline constexpr const char* SplineSubscriberConfigTypeId = "{44317FD2-51A1-41CA-BA44-F8BCAE9757CE}";

    inline constexpr const char* SplinePublisherComponentTypeId = "{29C02686-04F6-416D-8F47-D2456A3E114C}";
    inline constexpr const char* SplinePublisherConfigTypeId = "{DC7AC312-0F47-4EF2-A1B7-02E8716CF4EE}";
} // namespace SplineTools
