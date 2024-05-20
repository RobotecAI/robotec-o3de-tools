/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace LevelModificationTools
{
    // System Component TypeIds
    inline constexpr const char* LevelModificationToolsSystemComponentTypeId = "{AC900C65-CFC8-49A4-BF43-6F42169B78E1}";
    inline constexpr const char* LevelModificationToolsEditorSystemComponentTypeId = "{1C31A3BD-8EF4-49AD-8318-7C1D37629E9F}";

    // Module derived classes TypeIds
    inline constexpr const char* LevelModificationToolsModuleInterfaceTypeId = "{6CC3DDA1-5104-4124-9E35-03E54B83C8D9}";
    inline constexpr const char* LevelModificationToolsModuleTypeId = "{A34C7B0D-228C-4F0A-8A02-154C1378DB9E}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* LevelModificationToolsEditorModuleTypeId = LevelModificationToolsModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* LevelModificationToolsRequestsTypeId = "{7FFB3A2A-B9D6-4B5E-9B86-0105A7DDDC25}";
} // namespace LevelModificationTools
