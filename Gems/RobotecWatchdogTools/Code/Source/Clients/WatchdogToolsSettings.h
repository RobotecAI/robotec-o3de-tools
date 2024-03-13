/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/containers/unordered_map.h>

namespace WatchdogTools
{
    struct WatchdogSettings
    {
    public:
        AZ_RTTI(WatchdogSettings, "{760bc56f-f956-424c-8fcc-e19ad49b78d8}");

        WatchdogSettings() = default;
        virtual ~WatchdogSettings() = default;

        static void Reflect(AZ::ReflectContext* context);

        //! Read the watchdog configuration settings from the settings registry.
        //! This is done as a separate step from the constructor so that serialization has
        //! the ability to create a default set of values by using the default constructor.
        //! If we read these in the constructor, serialization would see all of the current values
        //! as default values and would try to prune them from the output by default.
        void LoadSettings(AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get());

        AZStd::vector<AZStd::string> m_requiredModules;
    };
} // namespace WatchdogTools
