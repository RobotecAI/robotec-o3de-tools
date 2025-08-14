/* Copyright 2025, Robotec.ai sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "WheelAnimToolModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include "Clients/WheelAnimComponent.h"
#include <WheelAnimTool/WheelAnimToolTypeIds.h>

namespace WheelAnimTool
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(WheelAnimToolModuleInterface, "WheelAnimToolModuleInterface", WheelAnimToolModuleInterfaceTypeId);

    AZ_RTTI_NO_TYPE_INFO_IMPL(WheelAnimToolModuleInterface, AZ::Module);

    AZ_CLASS_ALLOCATOR_IMPL(WheelAnimToolModuleInterface, AZ::SystemAllocator);

    WheelAnimToolModuleInterface::WheelAnimToolModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), { WheelAnimComponent::CreateDescriptor() });
    }

    AZ::ComponentTypeList WheelAnimToolModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{

        };
    }
} // namespace WheelAnimTool
