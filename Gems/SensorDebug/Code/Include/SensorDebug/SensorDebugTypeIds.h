
#pragma once

namespace SensorDebug
{
    // System Component TypeIds
    inline constexpr const char* SensorDebugSystemComponentTypeId = "{079B7923-E178-48AA-A0BD-9908BDB9F2BB}";
    inline constexpr const char* SensorDebugEditorSystemComponentTypeId = "{EEB09214-4772-4B21-8CF5-F29A9FF3CE1C}";

    // Module derived classes TypeIds
    inline constexpr const char* SensorDebugModuleInterfaceTypeId = "{6273F4BC-9B69-4567-A748-B602FBFCB6CE}";
    inline constexpr const char* SensorDebugModuleTypeId = "{28353925-BFF1-49A2-B003-5FC22530672D}";

    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* SensorDebugEditorModuleTypeId = SensorDebugModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* SensorDebugRequestsTypeId = "{E2FDB3D5-4F53-4CDD-B2EF-9F7D8D251155}";
} // namespace SensorDebug
