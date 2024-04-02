/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace WatchdogTools
{
    // System Component TypeIds
    inline constexpr const char* WatchdogToolsSystemComponentTypeId = "{5d02632d-52b2-40ec-9486-7bf0070aa58e}";
    inline constexpr const char* WatchdogToolsEditorSystemComponentTypeId = "{79bb5660-ef0a-4741-be29-e50b7dbf7d77}";

    // Module derived classes TypeIds
    inline constexpr const char* WatchdogToolsModuleInterfaceTypeId = "{511e88b8-e8ca-490d-b631-33ea7383c1f0}";
    inline constexpr const char* WatchdogToolsModuleTypeId = "{03618496-25ca-451b-8311-5ec2f0df103d}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* WatchdogToolsEditorModuleTypeId = WatchdogToolsModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* WatchdogToolsRequestsTypeId = "{fd2f4a25-0e90-4ecf-809c-eeb47983e693}";

} // namespace WatchdogTools
