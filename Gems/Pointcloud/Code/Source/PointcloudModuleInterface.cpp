/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <Pointcloud/PointcloudTypeIds.h>

#include "Tools/Components/PointcloudAssetBuilderSystemComponent.h"
#include <Clients/PointcloudSystemComponent.h>
namespace Pointcloud
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(PointcloudModuleInterface, "PointcloudModuleInterface", PointcloudModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(PointcloudModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(PointcloudModuleInterface, AZ::SystemAllocator);

    PointcloudModuleInterface::PointcloudModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                PointcloudSystemComponent::CreateDescriptor(),
                PointcloudAssetBuilderSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList PointcloudModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<PointcloudSystemComponent>(),
            azrtti_typeid<PointcloudAssetBuilderSystemComponent>(),
        };
    }
} // namespace Pointcloud
