
#pragma once

namespace FPSProfiler
{
    // System Component TypeIds
    inline constexpr const char* FPSProfilerComponentTypeId = "{B11CE88E-E1C5-404F-83D2-0D3850445A13}";
    inline constexpr const char* FPSProfilerEditorComponentTypeId = "{F4308920-CD0B-4A2E-91DE-2EC1E970F97A}";

    // Configs TypeIds
    inline constexpr const char* FPSProfilerConfigFileTypeId = "{68627A89-9426-4640-B460-63E6AA42CFBC}";
    inline constexpr const char* FPSProfilerConfigRecordTypeId = "{3CD7901E-F8C2-493A-B182-7EB2BAD9FBFB}";
    inline constexpr const char* FPSProfilerConfigPrecisionTypeId = "{0ADE9CF6-1FE2-49E5-AB8D-B240EBDB9C03}";
    inline constexpr const char* FPSProfilerConfigDebugTypeId = "{974F1627-C476-4310-B72A-7842BF868EC2}";

    // Module derived classes TypeIds
    inline constexpr const char* FPSProfilerModuleInterfaceTypeId = "{77EF155C-6E75-41B1-A939-AF5E2FE4FC6B}";
    inline constexpr const char* FPSProfilerModuleTypeId = "{2A84347B-7657-4BE1-BEA6-246ADB4F04FB}";

    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* FPSProfilerEditorModuleTypeId = FPSProfilerModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* FPSProfilerRequestsTypeId = "{D585EA71-B052-4C97-8647-4B3511CC7C5B}";
    inline constexpr const char* FPSProfilerNotificationsTypeId = "{63E04945-AD56-4BB6-888E-41C2FA71CC2F}";
} // namespace FPSProfiler
