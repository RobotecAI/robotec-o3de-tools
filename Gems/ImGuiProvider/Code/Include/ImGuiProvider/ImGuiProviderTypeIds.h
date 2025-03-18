
#pragma once

namespace ImGuiProvider
{
    // System Component TypeIds
    inline constexpr const char* ImGuiProviderSystemComponentTypeId = "{EC45C197-1A3E-44DB-BC61-4CB46BE33EB9}";
    inline constexpr const char* ImGuiProviderEditorSystemComponentTypeId = "{F587B9A3-7CC4-480A-BC7A-3065317F1B7A}";

    // Module derived classes TypeIds
    inline constexpr const char* ImGuiProviderModuleInterfaceTypeId = "{45D939AA-9177-47B7-9765-105EEEDABD84}";
    inline constexpr const char* ImGuiProviderModuleTypeId = "{2819D2AA-B3E6-4653-B108-502168902968}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ImGuiProviderEditorModuleTypeId = ImGuiProviderModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ImGuiProviderRequestsTypeId = "{C13938F9-A14D-4DE5-8833-6690DD1CA778}";
    inline constexpr const char* ImGuiProviderNotificationsTypeId = "{a382fe14-af02-4eb9-a1ad-31cee42b9d46}";
} // namespace ImGuiProvider
