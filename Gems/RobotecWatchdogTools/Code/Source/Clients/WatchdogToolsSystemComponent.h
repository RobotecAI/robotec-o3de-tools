/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/containers/set.h>
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

        AZ::Outcome<bool, AZStd::string> CheckRequiredModules() override;
        AZ::Outcome<bool, AZStd::string> VerifyComponentsLoaded() override;

    protected:
        // AZ::Component overrides ...
        void Activate() override;

        void Deactivate() override;

    private:
        AZStd::set<AZStd::string> GatherDynamicModules(const AZStd::vector<AZStd::string>& modules);
    };

} // namespace WatchdogTools
