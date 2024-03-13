/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/WatchdogToolsSystemComponent.h>

namespace WatchdogTools
{
    /// System component for WatchdogTools editor
    class WatchdogToolsEditorSystemComponent
        : public WatchdogToolsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = WatchdogToolsSystemComponent;

    public:
        AZ_COMPONENT_DECL(WatchdogToolsEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        WatchdogToolsEditorSystemComponent();
        ~WatchdogToolsEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace WatchdogTools
