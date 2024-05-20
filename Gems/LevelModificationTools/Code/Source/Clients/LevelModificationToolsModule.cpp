/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <LevelModificationTools/LevelModificationToolsTypeIds.h>
#include <LevelModificationToolsModuleInterface.h>

namespace LevelModificationTools
{
    class LevelModificationToolsModule : public LevelModificationToolsModuleInterface
    {
    public:
        AZ_RTTI(LevelModificationToolsModule, LevelModificationToolsModuleTypeId, LevelModificationToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(LevelModificationToolsModule, AZ::SystemAllocator);
    };
} // namespace LevelModificationTools

AZ_DECLARE_MODULE_CLASS(Gem_LevelModificationTools, LevelModificationTools::LevelModificationToolsModule)
