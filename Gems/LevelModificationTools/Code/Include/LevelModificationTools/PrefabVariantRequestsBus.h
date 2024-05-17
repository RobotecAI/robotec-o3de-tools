/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <LevelModificationTools/LevelModificationToolsTypeIds.h>

namespace LevelModificationTools
{
    class PrefabVariantRequests : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(PrefabVariantRequests, PrefabVariantRequestsTypeId);

        using BusIdType = AZ::s32;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        virtual ~PrefabVariantRequests() = default;

        //! Loads a prefab variant.
        //! @param variantId The id of the prefab variant to load, or -1 to empty.
        virtual void SetPrefabVariant(AZ::s32 variantId) = 0;

        static void Reflect(AZ::ReflectContext* context);
    };

    using PrefabVariantRequestsBus = AZ::EBus<PrefabVariantRequests>;

} // namespace LevelModificationTools
