/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Billboard/BillboardTypeIds.h>
#include <BillboardModuleInterface.h>
#include "BillboardSystemComponent.h"

#include <AzCore/RTTI/RTTI.h>


namespace Billboard
{
    class BillboardModule
        : public BillboardModuleInterface
    {
    public:
        AZ_RTTI(BillboardModule, BillboardModuleTypeId, BillboardModuleInterface);
        AZ_CLASS_ALLOCATOR(BillboardModule, AZ::SystemAllocator);

        BillboardModule()
        {
            m_descriptors.insert(m_descriptors.end(),
                {
                    BillboardSystemComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const
        {
            return AZ::ComponentTypeList{ azrtti_typeid<BillboardSystemComponent>() };
        }
    };
}// namespace Billboard

AZ_DECLARE_MODULE_CLASS(Gem_Billboard, Billboard::BillboardModule)
