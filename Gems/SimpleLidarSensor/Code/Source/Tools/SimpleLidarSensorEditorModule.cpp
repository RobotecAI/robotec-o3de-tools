/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include "SimpleLidarSensorEditorSystemComponent.h"
#include <SimpleLidarSensorModuleInterface.h>

namespace SimpleLidarSensor
{
    class SimpleLidarSensorEditorModule
        : public SimpleLidarSensorModuleInterface
    {
    public:
        AZ_RTTI(SimpleLidarSensorEditorModule, SimpleLidarSensorEditorModuleTypeId, SimpleLidarSensorModuleInterface);
        AZ_CLASS_ALLOCATOR(SimpleLidarSensorEditorModule, AZ::SystemAllocator);

        SimpleLidarSensorEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                SimpleLidarSensorEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
               azrtti_typeid<SimpleLidarSensorEditorSystemComponent>(),
            };
        }
    };
}// namespace SimpleLidarSensor

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), SimpleLidarSensor::SimpleLidarSensorEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_SimpleLidarSensor_Editor, SimpleLidarSensor::SimpleLidarSensorEditorModule)
#endif
