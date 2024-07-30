
#pragma once

namespace Smoothing
{
    // System Component TypeIds
    inline constexpr const char* SmoothingSystemComponentTypeId = "{FC39BBB4-C723-4994-9923-6EAF200891C1}";
    inline constexpr const char* SmoothingEditorSystemComponentTypeId = "{5B96FFF8-A18E-4373-B96A-56250A4ABA8B}";

    // Components, controllers and configs TypeIds
    inline constexpr const char* SmoothingComponentTypeId = "{2FAF0D38-4EEE-4351-96D5-089242B5D5B5}";
    inline constexpr const char* SmoothingComponentEditorComponentTypeId = "{A27AA1A4-9178-40B3-8079-F8486A7CE544}";
    inline constexpr const char* SmoothingComponentControllerTypeId = "{5A0DD7CE-1651-4751-8EEB-46FFFAD8E9EB}";
    inline constexpr const char* SmoothingConfigTypeId = "{142F6B91-F088-40DB-8409-7C2BE07E0286}";

    // Module derived classes TypeIds
    inline constexpr const char* SmoothingModuleInterfaceTypeId = "{86F9681F-2F29-486B-ABA7-B2152FB8A08C}";
    inline constexpr const char* SmoothingModuleTypeId = "{2B284F2F-270D-4CB2-956C-EC2B835D144E}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* SmoothingEditorModuleTypeId = SmoothingModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* SmoothingRequestsTypeId = "{897472A7-6396-476D-82E2-CD05FDCD681E}";
} // namespace Smoothing
