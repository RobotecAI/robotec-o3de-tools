/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "WatchdogToolsModuleInterface.h"
#include <AzCore/Memory/Memory.h>
#include <Clients/WatchdogToolsSystemComponent.h>
#include <WatchdogTools/WatchdogToolsTypeIds.h>

namespace WatchdogTools
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(WatchdogToolsModuleInterface, "WatchdogToolsModuleInterface", WatchdogToolsModuleInterfaceTypeId);

    AZ_RTTI_NO_TYPE_INFO_IMPL(WatchdogToolsModuleInterface, AZ::Module);

    AZ_CLASS_ALLOCATOR_IMPL(WatchdogToolsModuleInterface, AZ::SystemAllocator);

    WatchdogToolsModuleInterface::WatchdogToolsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), { WatchdogToolsSystemComponent::CreateDescriptor() });
    }

    AZ::ComponentTypeList WatchdogToolsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<WatchdogToolsSystemComponent>(),
        };
    }
} // namespace WatchdogTools
