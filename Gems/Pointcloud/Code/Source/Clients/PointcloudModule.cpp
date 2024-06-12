/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Pointcloud/PointcloudTypeIds.h>
#include <PointcloudModuleInterface.h>
#include "PointcloudSystemComponent.h"

#include <AzCore/RTTI/RTTI.h>


namespace Pointcloud
{
    class PointcloudModule
        : public PointcloudModuleInterface
    {
    public:
        AZ_RTTI(PointcloudModule, PointcloudModuleTypeId, PointcloudModuleInterface);
        AZ_CLASS_ALLOCATOR(PointcloudModule, AZ::SystemAllocator);

        PointcloudModule()
        {
            m_descriptors.insert(m_descriptors.end(),
                {
                    PointcloudSystemComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const
        {
            return AZ::ComponentTypeList{ azrtti_typeid<PointcloudSystemComponent>() };
        }
    };
}// namespace Pointcloud

AZ_DECLARE_MODULE_CLASS(Gem_Pointcloud, Pointcloud::PointcloudModule)
