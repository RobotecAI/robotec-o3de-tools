/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include <SimpleLidarSensorModuleInterface.h>

namespace SimpleLidarSensor
{
    class SimpleLidarSensorModule
        : public SimpleLidarSensorModuleInterface
    {
    public:
        AZ_RTTI(SimpleLidarSensorModule, SimpleLidarSensorModuleTypeId, SimpleLidarSensorModuleInterface);
        AZ_CLASS_ALLOCATOR(SimpleLidarSensorModule, AZ::SystemAllocator);
    };
}// namespace SimpleLidarSensor

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), SimpleLidarSensor::SimpleLidarSensorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_SimpleLidarSensor, SimpleLidarSensor::SimpleLidarSensorModule)
#endif
