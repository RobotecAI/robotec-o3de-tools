/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <LevelModificationTools/LevelModificationToolsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace LevelModificationTools
{
    class LevelModificationToolsRequests
    {
    public:
        AZ_RTTI(LevelModificationToolsRequests, LevelModificationToolsRequestsTypeId);
        virtual ~LevelModificationToolsRequests() = default;
        // Put your public methods here
    };

    class LevelModificationToolsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using LevelModificationToolsRequestBus = AZ::EBus<LevelModificationToolsRequests, LevelModificationToolsBusTraits>;
    using LevelModificationToolsInterface = AZ::Interface<LevelModificationToolsRequests>;

} // namespace LevelModificationTools
