/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Pointcloud/PointcloudTypeIds.h>
#include <PointcloudModuleInterface.h>
#include "PointcloudEditorSystemComponent.h"
#include "Components/PointcloudEditorComponent.h"
#include "Tools/Components/PointcloudAssetBuilderSystemComponent.h"
namespace Pointcloud
{
    class PointcloudEditorModule
        : public PointcloudModuleInterface
    {
    public:
        AZ_RTTI(PointcloudEditorModule, PointcloudEditorModuleTypeId, PointcloudModuleInterface);
        AZ_CLASS_ALLOCATOR(PointcloudEditorModule, AZ::SystemAllocator);

        PointcloudEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                PointcloudEditorSystemComponent::CreateDescriptor(),
                PointcloudEditorComponent::CreateDescriptor(),
                PointcloudAssetBuilderSystemComponent::CreateDescriptor()
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<PointcloudEditorSystemComponent>(),
            };
        }
    };
}// namespace Pointcloud

AZ_DECLARE_MODULE_CLASS(Gem_Pointcloud, Pointcloud::PointcloudEditorModule)
