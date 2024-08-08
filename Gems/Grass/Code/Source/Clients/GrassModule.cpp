/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Grass/GrassTypeIds.h>
#include <GrassModuleInterface.h>
#include "GrassSystemComponent.h"

#include <AzCore/RTTI/RTTI.h>


namespace Grass
{
    class GrassModule
        : public GrassModuleInterface
    {
    public:
        AZ_RTTI(GrassModule, GrassModuleTypeId, GrassModuleInterface);
        AZ_CLASS_ALLOCATOR(GrassModule, AZ::SystemAllocator);

        GrassModule()
        {
            m_descriptors.insert(m_descriptors.end(),
                {
                    GrassSystemComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const
        {
            return AZ::ComponentTypeList{ azrtti_typeid<GrassSystemComponent>() };
        }
    };
}// namespace Grass

AZ_DECLARE_MODULE_CLASS(Gem_Grass, Grass::GrassModule)
