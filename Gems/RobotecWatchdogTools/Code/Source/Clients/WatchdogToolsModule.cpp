/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "WatchdogToolsSystemComponent.h"
#include <WatchdogTools/WatchdogToolsTypeIds.h>
#include <WatchdogToolsModuleInterface.h>

namespace WatchdogTools
{
    class WatchdogToolsModule : public WatchdogToolsModuleInterface
    {
    public:
        AZ_RTTI(WatchdogToolsModule, WatchdogToolsModuleTypeId, WatchdogToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(WatchdogToolsModule, AZ::SystemAllocator);
    };
} // namespace WatchdogTools

AZ_DECLARE_MODULE_CLASS(Gem_WatchdogTools, WatchdogTools::WatchdogToolsModule)
