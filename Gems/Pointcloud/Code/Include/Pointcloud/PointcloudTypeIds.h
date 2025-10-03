/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace Pointcloud
{
    // System Component TypeIds
    inline constexpr const char* PointcloudSystemComponentTypeId = "{3EBCAD0C-39B7-4782-B9CB-D62F35331D87}";
    inline constexpr const char* PointcloudEditorSystemComponentTypeId = "{88835E98-561B-4D74-B7C4-C539E2950E8D}";

    // Module derived classes TypeIds
    inline constexpr const char* PointcloudModuleInterfaceTypeId = "{8077C268-B445-409C-9BBA-DE557794B240}";
    inline constexpr const char* PointcloudModuleTypeId = "{7BFFAD35-81E5-40FF-BE8D-19B0A039266C}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* PointcloudEditorModuleTypeId = PointcloudModuleTypeId;

    // Components TypeIds
    inline constexpr const char* PointcloudComponentTypeId = "{0190C091-83AA-7C6E-A6DA-5EFEA1F23473}";
    inline constexpr const char* PointcloudEditorComponentTypeId = "{018FBA15-560F-78CB-AFB4-CF4D00CEFC17}";
    inline constexpr const char* PointcloudComponentConfigTypeId = "{3AE848A0-3CD0-439E-BAFE-DB0270CAAE47}";

    // Assets TypeIds
    inline constexpr const char* PointcloudAssetTypeId = "{0190C039-385B-7C8A-9172-31E83C091216}";
    inline constexpr const char* CloudVertexAssetTypeId = "{32EAC666-ED12-4233-B033-BA2B38A8582C}";

    // Interface TypeIds
    inline constexpr const char* PointcloudRequestsTypeId = "{86CA76D8-2225-4C50-86E4-B1C8EFDEA8EF}";
    // component buses
    inline constexpr const char* PointCloudRequestsTypeId = "{5985443B-FEAB-444F-A633-4B2A8142C560}";
} // namespace Pointcloud
