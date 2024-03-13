/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <WatchdogTools/WatchdogToolsBus.h>

namespace WatchdogTools
{
    class WatchdogToolsSystemComponent
        : public AZ::Component
        , protected WatchdogToolsRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(WatchdogToolsSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        WatchdogToolsSystemComponent();

        ~WatchdogToolsSystemComponent();

    protected:
        // AZ::Component overrides ...
        // void Init() override;

        void Activate() override;

        void Deactivate() override;
    };

} // namespace WatchdogTools
