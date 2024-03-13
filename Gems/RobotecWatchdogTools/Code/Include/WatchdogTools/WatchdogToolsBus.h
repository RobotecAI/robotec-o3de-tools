/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <WatchdogTools/WatchdogToolsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace WatchdogTools
{
    class WatchdogToolsRequests
    {
    public:
        AZ_RTTI(WatchdogToolsRequests, WatchdogToolsRequestsTypeId);
        virtual ~WatchdogToolsRequests() = default;
        // Put your public methods here
    };

    class WatchdogToolsBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using WatchdogToolsRequestBus = AZ::EBus<WatchdogToolsRequests, WatchdogToolsBusTraits>;
    using WatchdogToolsInterface = AZ::Interface<WatchdogToolsRequests>;

} // namespace WatchdogTools
