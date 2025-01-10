
#pragma once

namespace ImGuizmoGem
{
    // System Component TypeIds
    inline constexpr const char* ImGuizmoGemSystemComponentTypeId = "{D1345DE6-1ABE-4549-AA32-FA7B48195030}";
    inline constexpr const char* ImGuizmoGemEditorSystemComponentTypeId = "{C433311E-A8A5-43B1-91F8-24A98869A4F6}";

    // Module derived classes TypeIds
    inline constexpr const char* ImGuizmoGemModuleInterfaceTypeId = "{4005F59E-0F60-4D3E-8913-450DE057D0C3}";
    inline constexpr const char* ImGuizmoGemModuleTypeId = "{259CBE69-6577-47E4-B3BE-284E9C56B2E0}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ImGuizmoGemEditorModuleTypeId = ImGuizmoGemModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ImGuizmoGemRequestsTypeId = "{C27C7392-3C1E-439A-8DDF-658A5684DBBC}";
} // namespace ImGuizmoGem
