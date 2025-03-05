
#pragma once

namespace FPSProfiler
{
    // System Component TypeIds
    inline constexpr const char* FPSProfilerSystemComponentTypeId = "{B11CE88E-E1C5-404F-83D2-0D3850445A13}";
    inline constexpr const char* FPSProfilerEditorSystemComponentTypeId = "{F4308920-CD0B-4A2E-91DE-2EC1E970F97A}";
    inline constexpr const char* FPSProfilerConfigFileTypeId = "{70857242-4363-403C-ACF1-4A401B1024B5}";

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
