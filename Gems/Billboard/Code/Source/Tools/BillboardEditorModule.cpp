/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Billboard/BillboardTypeIds.h>
#include <BillboardModuleInterface.h>
#include "BillboardEditorSystemComponent.h"
#include "Components/BillboardEditorComponent.h"

namespace Billboard
{
    class BillboardEditorModule
        : public BillboardModuleInterface
    {
    public:
        AZ_RTTI(BillboardEditorModule, BillboardEditorModuleTypeId, BillboardModuleInterface);
        AZ_CLASS_ALLOCATOR(BillboardEditorModule, AZ::SystemAllocator);

        BillboardEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                BillboardEditorSystemComponent::CreateDescriptor(),
                BillboardEditorComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<BillboardEditorSystemComponent>(),
            };
        }
    };
}// namespace Billboard

AZ_DECLARE_MODULE_CLASS(Gem_Billboard, Billboard::BillboardEditorModule)
