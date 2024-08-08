/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Grass/GrassTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace Grass
{
    class GrassRequests
    {
    public:
        AZ_RTTI(GrassRequests, GrassRequestsTypeId);
        virtual ~GrassRequests() = default;
        // Put your public methods here
    };

    class GrassBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using GrassRequestBus = AZ::EBus<GrassRequests, GrassBusTraits>;
    using GrassInterface = AZ::Interface<GrassRequests>;

} // namespace Grass
