/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LevelModificationToolsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <LevelModificationTools/LevelModificationToolsTypeIds.h>

namespace LevelModificationTools
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(LevelModificationToolsModuleInterface,
        "LevelModificationToolsModuleInterface", LevelModificationToolsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(LevelModificationToolsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(LevelModificationToolsModuleInterface, AZ::SystemAllocator);

    LevelModificationToolsModuleInterface::LevelModificationToolsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            });
    }

    AZ::ComponentTypeList LevelModificationToolsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
        };
    }
} // namespace LevelModificationTools
