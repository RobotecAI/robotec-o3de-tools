/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <LevelModificationTools/PrefabVariantRequestsBus.h>

namespace LevelModificationTools
{
    void PrefabVariantRequests::Reflect(AZ::ReflectContext* context)
    {
        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<PrefabVariantRequestsBus>("PrefabVariantRequestsBus")
                ->Attribute(AZ::Script::Attributes::Category, "LevelModificationTools")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "LevelModificationTools")
                ->Event(
                    "SetPrefabVariant",
                    &PrefabVariantRequestsBus::Events::SetPrefabVariant,
                    { { { "Variant", "Number of variant to load, or -1 to clear" } } });
        }
    }
} // namespace LevelModificationTools
