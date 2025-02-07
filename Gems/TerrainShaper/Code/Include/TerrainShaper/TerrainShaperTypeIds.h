
#pragma once

namespace TerrainShaper
{
    // System Component TypeIds
    inline constexpr const char* TerrainShaperSystemComponentTypeId = "{87B6976B-1F0E-40D6-8FD1-ED8A4543083F}";
    inline constexpr const char* TerrainShaperEditorSystemComponentTypeId = "{2C7740AD-DBE7-45DB-B73A-F9D95C197481}";

    // Module derived classes TypeIds
    inline constexpr const char* TerrainShaperModuleInterfaceTypeId = "{41BDF84B-0EA3-4BDB-8790-E2B4F323AA40}";
    inline constexpr const char* TerrainShaperModuleTypeId = "{D4015170-8045-43FB-A0F4-999A1637BCEA}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* TerrainShaperEditorModuleTypeId = TerrainShaperModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* TerrainShaperRequestsTypeId = "{82269232-BDEC-4286-85A4-C9E5CBC84778}";
} // namespace TerrainShaper
