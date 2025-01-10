
#pragma once

namespace ImGuizmo
{
    // System Component TypeIds
    inline constexpr const char* ImGuizmoSystemComponentTypeId = "{A242CA3D-3F28-4053-8F00-0FD41305BE6F}";
    inline constexpr const char* ImGuizmoEditorSystemComponentTypeId = "{5C3AAB3A-216E-45B2-B803-E6C1E6A06C48}";

    // Module derived classes TypeIds
    inline constexpr const char* ImGuizmoModuleInterfaceTypeId = "{62781671-6956-439F-B27E-26385519A967}";
    inline constexpr const char* ImGuizmoModuleTypeId = "{6A78AD5B-0E26-4B79-A5BA-01116F1A3F88}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ImGuizmoEditorModuleTypeId = ImGuizmoModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ImGuizmoRequestsTypeId = "{3922437E-28BB-4D7D-A655-3398F52E9F75}";
} // namespace ImGuizmo
