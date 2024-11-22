
#pragma once

namespace GeoJSONSpawner
{
    // System Component TypeIds
    inline constexpr const char* GeoJSONSpawnerSystemComponentTypeId = "{CD4EB5DE-E197-433A-B651-A0507E6A7138}";
    inline constexpr const char* GeoJSONSpawnerEditorSystemComponentTypeId = "{6894C22F-7C51-4C87-92E0-83D93F42252C}";

    // Module derived classes TypeIds
    inline constexpr const char* GeoJSONSpawnerModuleInterfaceTypeId = "{457EB65D-5DFC-4341-BDE3-79D054485C58}";
    inline constexpr const char* GeoJSONSpawnerModuleTypeId = "{E56013BF-CE3A-4AA5-81AD-698844BEA7C6}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* GeoJSONSpawnerEditorModuleTypeId = GeoJSONSpawnerModuleTypeId;

    // Editor Components TypeIds
    inline constexpr const char* GeoJSONSpawnerEditorComponentTypeId = "{97d8cdeb-8a82-4bb0-adc3-c42a3eac21c7}";

    // Configurations TypeIds
    inline constexpr const char* GeoJSONSpawnerConfigurationTypeId = "{cee254bb-edc8-45b1-b85a-aaf19ec3af83}";

    // Components TypeIds
    inline constexpr const char* GeoJSONSpawnerComponentTypeId = "{839ede69-92f1-45b0-a60e-035d0b84e1fd}";

    // Interface TypeIds
    inline constexpr const char* GeoJSONSpawnerRequestsTypeId = "{EE523E9E-CFDF-4590-BBDE-054848BBA790}";
} // namespace GeoJSONSpawner
