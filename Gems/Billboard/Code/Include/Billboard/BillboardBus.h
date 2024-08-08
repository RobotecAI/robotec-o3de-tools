/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Billboard/BillboardTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace Billboard
{
    class BillboardRequests
    {
    public:
        AZ_RTTI(BillboardRequests, BillboardRequestsTypeId);
        virtual ~BillboardRequests() = default;
        // Put your public methods here
    };

    class BillboardBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using BillboardRequestBus = AZ::EBus<BillboardRequests, BillboardBusTraits>;
    using BillboardInterface = AZ::Interface<BillboardRequests>;

} // namespace Billboard
