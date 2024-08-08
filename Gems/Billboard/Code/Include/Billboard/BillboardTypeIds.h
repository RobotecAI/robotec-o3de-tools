/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace Billboard
{
    // System Component TypeIds
    inline constexpr const char* BillboardSystemComponentTypeId = "{3EBCAD0C-39B7-4782-B9CB-D62F35331D11}";
    inline constexpr const char* BillboardEditorSystemComponentTypeId = "{88835E98-561B-4D74-B7C4-C539E2950E11}";

    // Module derived classes TypeIds
    inline constexpr const char* BillboardModuleInterfaceTypeId = "{8077C268-B445-409C-9BBA-DE557794B211}";
    inline constexpr const char* BillboardModuleTypeId = "{7BFFAD35-81E5-40FF-BE8D-19B0A0392611}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* BillboardEditorModuleTypeId = BillboardModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* BillboardRequestsTypeId = "{86CA76D8-2225-4C50-86E4-B1C8EFDEA811}";
} // namespace Billboard
