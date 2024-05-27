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

    // Interface TypeIds
    inline constexpr const char* PointcloudRequestsTypeId = "{86CA76D8-2225-4C50-86E4-B1C8EFDEA8EF}";
} // namespace Pointcloud
