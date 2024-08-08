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
    inline constexpr const char* GrassSystemComponentTypeId = "{e0bd4331-bbc5-454c-9a93-a2a48bdcbb60}";
    inline constexpr const char* GrassEditorSystemComponentTypeId = "{d7053054-ce68-4449-b9a9-3308cbf4eccd}";
    inline constexpr const char* GrassComponentTypeId =  "{fb7551b4-25ca-4f14-b633-a97f48324ae7}";
    // Module derived classes TypeIds
    inline constexpr const char* GrassModuleInterfaceTypeId = "{7d99ed0c-7491-41ab-83a6-53509b3fc2c8}";
    inline constexpr const char* GrassModuleTypeId = "{f4e977c4-1a5f-4f49-8254-f82ad4ca111e}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* GrassEditorModuleTypeId = GrassModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* GrassRequestsTypeId = "{310badef-c77d-42f1-a315-9eeaf5ce67c9}";
} // namespace Grass
