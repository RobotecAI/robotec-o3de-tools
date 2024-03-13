/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "WatchdogToolsEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <WatchdogTools/WatchdogToolsTypeIds.h>

namespace WatchdogTools
{
    AZ_COMPONENT_IMPL(
        WatchdogToolsEditorSystemComponent,
        "WatchdogToolsEditorSystemComponent",
        WatchdogToolsEditorSystemComponentTypeId,
        BaseSystemComponent);

    void WatchdogToolsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<WatchdogToolsEditorSystemComponent, WatchdogToolsSystemComponent>()->Version(0);
        }
    }

    WatchdogToolsEditorSystemComponent::WatchdogToolsEditorSystemComponent() = default;

    WatchdogToolsEditorSystemComponent::~WatchdogToolsEditorSystemComponent() = default;

    void WatchdogToolsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("WatchdogToolsEditorService"));
    }

    void WatchdogToolsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("WatchdogToolsEditorService"));
    }

    void WatchdogToolsEditorSystemComponent::Activate()
    {
        WatchdogToolsSystemComponent::Activate();
        AZ_Trace("Watchdog", "WatchdogTools Editor Component is being activated");

        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void WatchdogToolsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        WatchdogToolsSystemComponent::Deactivate();
    }

} // namespace WatchdogTools
