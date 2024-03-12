/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Clients/WatchdogToolsSettings.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/Settings/SettingsRegistryVisitorUtils.h>

namespace WatchdogTools
{
    namespace
    {
        struct WatchdogSettingsRootKeyType
        {
            using StringType = AZStd::fixed_string<256>;

            constexpr StringType operator()(AZStd::string_view name) const
            {
                constexpr size_t MaxTotalKeySize = StringType{}.max_size();
                // The +1 is for the '/' separator
                [[maybe_unused]] const size_t maxNameSize = MaxTotalKeySize - (SettingsPrefix.size() + 1);

                AZ_Assert(
                    name.size() <= maxNameSize,
                    R"(The size of the event logger name "%.*s" is too long. It must be <= %zu characters)",
                    AZ_STRING_ARG(name),
                    maxNameSize);
                StringType settingsKey(SettingsPrefix);
                settingsKey += '/';
                settingsKey += name;

                return settingsKey;
            }

            constexpr operator AZStd::string_view() const
            {
                return SettingsPrefix;
            }

        private:
            AZStd::string_view SettingsPrefix = "/O3DE/Watchdog";
        };

        constexpr WatchdogSettingsRootKeyType WatchdogSettingsRootKey;

        constexpr auto WatchdogRequiredModulesRegistryKey = WatchdogSettingsRootKey("RequiredModules");
    } // namespace

    void WatchdogSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<WatchdogSettings>()->Version(0)->Field("RequiredModules", &WatchdogSettings::m_requiredModules);

            if (auto editContext = serializeContext->GetEditContext(); editContext != nullptr)
            {
                editContext->Class<WatchdogSettings>("Watchdog Tools Settings", "Exposes settings which alters watchdog behavior.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WatchdogSettings::m_requiredModules,
                        "Required Modules",
                        "List of module name prefixes that are required for the project to run.");
            }
        }
    }

    void WatchdogSettings::LoadSettings(AZ::SettingsRegistryInterface* settingsRegistry)
    {
        if (settingsRegistry == nullptr)
        {
            AZ_Warning("WatchdogSettings", false, "No settings registry provided, default settings will be used.");
            return ;
        }

        // Callback is needed for visiting array elements.
        auto CollectRequiredModules = [&settingsRegistry, this](const AZ::SettingsRegistryInterface::VisitArgs& visitArgs)
        {
            AZ_Trace("WatchdogSettings", "Visiting: " AZ_STRING_FORMAT, AZ_STRING_ARG(visitArgs.m_jsonKeyPath));
            if (AZ::SettingsRegistryInterface::FixedValueString value; settingsRegistry->Get(value, visitArgs.m_jsonKeyPath))
            {
                // Ignore empty entries.
                if (value.empty())
                {
                    return AZ::SettingsRegistryInterface::VisitResponse::Continue;
                }

                AZ_Trace("WatchdogSettings", "Value added to list of required modules: %s", value.c_str());
                m_requiredModules.push_back(value.c_str());
            }
            return AZ::SettingsRegistryInterface::VisitResponse::Continue;
        };
        bool result = AZ::SettingsRegistryVisitorUtils::VisitArray(
            *settingsRegistry, CollectRequiredModules, WatchdogRequiredModulesRegistryKey);
        AZ_Warning("WatchdogSettings", result, "Required modules not read. Use defaults");
    }
} // namespace WatchdogTools
