/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace Grass
{
    // System Component TypeIds
    inline constexpr const char* GrassSystemComponentTypeId = "{747f9201-a9cb-4636-965f-edff04dfffed}";
    inline constexpr const char* GrassEditorSystemComponentTypeId = "{bd3aa3d4-c3d9-4149-8dbf-e58667fff13b}";

    // Module derived classes TypeIds
    inline constexpr const char* GrassModuleInterfaceTypeId = "{73e4f133-2287-4970-ac6b-1991d977280a}";
    inline constexpr const char* GrassModuleTypeId = "{91cd974f-87de-4708-b4f6-b96f1523723c}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* GrassEditorModuleTypeId = GrassModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* GrassRequestsTypeId = "{d504d3b9-52e0-4197-a54a-cc28200c31e5}";
} // namespace Grass
