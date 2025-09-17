/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace SimpleLidarSensor
{
    class SimpleLidarSensorRequests
    {
    public:
        AZ_RTTI(SimpleLidarSensorRequests, SimpleLidarSensorRequestsTypeId);
        virtual ~SimpleLidarSensorRequests() = default;
        // Put your public methods here
    };

    class SimpleLidarSensorBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SimpleLidarSensorRequestBus = AZ::EBus<SimpleLidarSensorRequests, SimpleLidarSensorBusTraits>;
    using SimpleLidarSensorInterface = AZ::Interface<SimpleLidarSensorRequests>;

} // namespace SimpleLidarSensor
