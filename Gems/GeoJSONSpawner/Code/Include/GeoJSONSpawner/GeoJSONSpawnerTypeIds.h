/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

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
    inline constexpr const char* GeoJSONSpawnableAssetConfigurationTypeId = "{cee254bb-edc8-45b1-b85a-aaf19ec3af83}";
    inline constexpr const char* GeoJSONSpawnableEntityInfoTypeId = "{170bb487-fbae-4394-8176-069045a3a316}";
    inline constexpr const char* FeatureObjectInfoTypeId = "{00f38302-9f4d-4bac-805b-72a29e42a704}";
    inline constexpr const char* GeoJSONSpawnerEditorTerrainSettingsConfigTypeId = "{9bb9e1c6-dd7c-435f-8fe1-bd5498c3e8f2}";

    // Wrapper TypeIds
    inline constexpr const char* GeoJSONSpawnerSpawnTicketMapWrapperTypeId = "{714316F3-190D-4189-B14B-B601BE22EEBC}";

    // Components TypeIds
    inline constexpr const char* GeoJSONSpawnerComponentTypeId = "{839ede69-92f1-45b0-a60e-035d0b84e1fd}";

    // Interface TypeIds
    inline constexpr const char* GeoJSONSpawnerRequestsTypeId = "{EE523E9E-CFDF-4590-BBDE-054848BBA790}";
    inline constexpr const char* GeoJSONSpawnerInterfaceTypeId = "{4770B38A-D018-4184-A672-3F9155B7BEE7}";
    inline constexpr const char* GeoJSONSpawnerNotificationBusHandlerTypeId = "{9D8C5E99-7151-4AA4-9B5F-2D8AE2F097F7}";

    // Enums
    inline constexpr const char* SpawnDespawnStatusTypeId = "{1E22A24E-8365-4CC7-8B99-CC0EF0B25B13}";
} // namespace GeoJSONSpawner
