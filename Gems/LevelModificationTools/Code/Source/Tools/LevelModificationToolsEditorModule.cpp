/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabVariantEditorComponent.h"
#include <LevelModificationTools/LevelModificationToolsTypeIds.h>
#include <LevelModificationToolsModuleInterface.h>
namespace LevelModificationTools
{
    class LevelModificationToolsEditorModule : public LevelModificationToolsModuleInterface
    {
    public:
        AZ_RTTI(LevelModificationToolsEditorModule, LevelModificationToolsEditorModuleTypeId, LevelModificationToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(LevelModificationToolsEditorModule, AZ::SystemAllocator);

        LevelModificationToolsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    PrefabVariantEditorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{};
        }
    };
} // namespace LevelModificationTools

AZ_DECLARE_MODULE_CLASS(Gem_LevelModificationTools, LevelModificationTools::LevelModificationToolsEditorModule)
