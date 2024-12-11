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

namespace GeoJSONSpawnerROS2
{
    // System Component TypeIds
    inline constexpr const char* GeoJSONSpawnerROS2SystemComponentTypeId = "{715CB9C6-1EFD-469D-B7DC-72D097E1FBE9}";
    inline constexpr const char* GeoJSONSpawnerROS2EditorSystemComponentTypeId = "{F56F94B4-D27B-4466-9794-3ECC2D35F5E9}";

    // Module derived classes TypeIds
    inline constexpr const char* GeoJSONSpawnerROS2ModuleInterfaceTypeId = "{F8EFEBC9-E997-4830-A035-AF7A201F42A9}";
    inline constexpr const char* GeoJSONSpawnerROS2ModuleTypeId = "{4F1BA299-BF1E-4810-B636-4896CC2842E8}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* GeoJSONSpawnerROS2EditorModuleTypeId = GeoJSONSpawnerROS2ModuleTypeId;

    // Editor Components TypeIds
    inline constexpr const char* GeoJSONSpawnerROS2EditorComponentTypeId = "{e73adfb4-d45e-49de-a863-b20877300e99}";

    // Configurations TypeIds
    inline constexpr const char* GeoJSONSpawnerROS2ConfigurationTypeId = "{a023282f-5ec5-4af1-902d-74772f024339}";

    // Components TypeIds
    inline constexpr const char* GeoJSONSpawnerROS2TypeId = "{a302588e-10c5-443b-a9ec-5edc33c56ef2}";

    // Interface TypeIds
    inline constexpr const char* GeoJSONSpawnerROS2RequestsTypeId = "{636D9D5F-7EA7-47CA-902E-7DE5003A7FC7}";
} // namespace GeoJSONSpawnerROS2
