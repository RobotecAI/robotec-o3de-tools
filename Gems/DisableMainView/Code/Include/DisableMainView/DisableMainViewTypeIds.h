
#pragma once

namespace DisableMainView {
    // System Component TypeIds
    inline constexpr const char *DisableMainViewSystemComponentTypeId = "{9EFB170F-4263-433D-A4D8-8A2637453E80}";
    inline constexpr const char *DisableMainViewEditorSystemComponentTypeId = "{BBCD6804-F0A2-4335-9E5B-45CA1DEB7A32}";

    // Module derived classes TypeIds
    inline constexpr const char *DisableMainViewModuleInterfaceTypeId = "{B8353C65-ECBB-4B02-8625-B789517D00FC}";
    inline constexpr const char *DisableMainViewModuleTypeId = "{D987D085-36E1-4F84-A209-1BAE2FDBCB0E}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char *DisableMainViewEditorModuleTypeId = DisableMainViewModuleTypeId;

    // Interface TypeIds
    inline constexpr const char *DisableMainViewRequestsTypeId = "{8942E4F4-AA58-4DC9-9132-0CEED5BB928F}";
} // namespace DisableMainView
