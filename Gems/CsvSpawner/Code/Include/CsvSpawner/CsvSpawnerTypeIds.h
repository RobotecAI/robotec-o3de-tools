
#pragma once

namespace CsvSpawner
{
    // System Component TypeIds
    inline constexpr const char* CsvSpawnerSystemComponentTypeId = "{E05A4789-6293-4411-A5C9-E7FC26CE3247}";
    inline constexpr const char* CsvSpawnerEditorSystemComponentTypeId = "{8797F3DE-93E6-433E-AC39-D9C74D4DEE57}";

    // Module derived classes TypeIds
    inline constexpr const char* CsvSpawnerModuleInterfaceTypeId = "{17B9C031-6510-4540-AD90-DA17792EA9C7}";
    inline constexpr const char* CsvSpawnerModuleTypeId = "{E44EEBCC-5048-46BE-87A0-C43E0093D256}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* CsvSpawnerEditorModuleTypeId = CsvSpawnerModuleTypeId;

    // Component TypeIds
    inline constexpr const char* CsvSpawnableEntityInfoTypeId = "{6acce47e-9cac-4a6e-8726-ea614fb2d30d}";
    inline constexpr const char* CsvSpawnableAssetConfigurationTypeId = "{d04ea4de-b4dc-4e76-b742-c1d77203bc3e}";
    inline constexpr const char* CsvSpawnerEditorComponentTypeId = "{8b016a40-c585-48a1-a3e5-2814f599418c}";
    inline constexpr const char* CsvSpawnerComponentTypeId = "{59b31372-1f3c-4733-b61b-0fe94b5a8f3e}";

    // Interface TypeIds
    inline constexpr const char* CsvSpawnerInterfaceTypeId = "{77ACBD4E-069E-4610-8154-E1AC28CEE05A}";
    inline constexpr const char* CsvSpawnerSpawnInfoTypeId = "{81E5A014-3232-4359-98F5-7F9D7152629E}";
} // namespace CsvSpawner
