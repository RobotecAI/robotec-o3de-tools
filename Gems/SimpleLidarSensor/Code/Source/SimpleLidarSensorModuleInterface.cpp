/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimpleLidarSensorModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include "Clients/SimpleLidar.h"
#include "Clients/SimpleLidarSensorSystemComponent.h"
#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>

namespace SimpleLidarSensor
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(SimpleLidarSensorModuleInterface,
        "SimpleLidarSensorModuleInterface", SimpleLidarSensorModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(SimpleLidarSensorModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(SimpleLidarSensorModuleInterface, AZ::SystemAllocator);

    SimpleLidarSensorModuleInterface::SimpleLidarSensorModuleInterface()
    {
        m_descriptors.insert(m_descriptors.end(), {
            SimpleLidarSensor::SimpleLidar::CreateDescriptor(),
            SimpleLidarSensor::SimpleLidarSensorSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList SimpleLidarSensorModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<SimpleLidarSensorSystemComponent>(),
        };
    }
} // namespace SimpleLidarSensor
