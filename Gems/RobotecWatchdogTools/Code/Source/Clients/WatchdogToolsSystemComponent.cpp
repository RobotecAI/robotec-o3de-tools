/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "WatchdogToolsSystemComponent.h"

#include "AzCore/Module/DynamicModuleHandle.h"
#include "AzCore/Module/ModuleManagerBus.h"
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Clients/WatchdogToolsSettings.h>
#include <WatchdogTools/WatchdogToolsTypeIds.h>

namespace WatchdogTools
{
    AZ_COMPONENT_IMPL(WatchdogToolsSystemComponent, "WatchdogToolsSystemComponent", WatchdogToolsSystemComponentTypeId);

    void WatchdogToolsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        WatchdogSettings::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<WatchdogToolsSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void WatchdogToolsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("WatchdogToolsService"));
    }

    void WatchdogToolsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("WatchdogToolsService"));
    }

    WatchdogToolsSystemComponent::WatchdogToolsSystemComponent()
    {
        if (WatchdogToolsInterface::Get() == nullptr)
        {
            WatchdogToolsInterface::Register(this);
        }
    }

    WatchdogToolsSystemComponent::~WatchdogToolsSystemComponent()
    {
        if (WatchdogToolsInterface::Get() == this)
        {
            WatchdogToolsInterface::Unregister(this);
        }
    }

    void WatchdogToolsSystemComponent::Activate()
    {
        WatchdogToolsRequestBus::Handler::BusConnect();
#if !defined(AZ_MONOLITHIC_BUILD)
        CheckRequiredModules();
#endif
    }

    void WatchdogToolsSystemComponent::Deactivate()
    {
        WatchdogToolsRequestBus::Handler::BusDisconnect();
    }

    void WatchdogToolsSystemComponent::CheckRequiredModules()
    {
        AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
        AZ_Assert(settingsRegistry, "Settings Registry Interface not available");
        WatchdogSettings watchdogSettings;
        watchdogSettings.LoadSettings(settingsRegistry);
        AZStd::set<AZStd::string> requiredDynamicModules = GatherDynamicModules(watchdogSettings.m_requiredModules);
        AZ::ModuleManagerRequestBus::Broadcast(
            &AZ::ModuleManagerRequestBus::Events::EnumerateModules,
            [&requiredDynamicModules](const AZ::ModuleData& moduleData)
            {
                // // We can only enumerate shared libs, static libs are invisible to us
                auto handle = moduleData.GetDynamicModuleHandle();
                if (!handle)
                {
                    return true;
                }
                bool isLoaded = moduleData.GetDynamicModuleHandle()->IsLoaded();
                if (!isLoaded)
                {
                    AZ_Warning("Watchdog", false, "Module %s is NOT loaded ", moduleData.GetDebugName());
                    return false;
                }
                AZStd::string_view moduleName = moduleData.GetDebugName();

                requiredDynamicModules.erase(moduleName);

                return true;
            });

        if (!requiredDynamicModules.empty())
        {
            for (auto nonLoadedModule : requiredDynamicModules)
            {
                AZ_Printf("Watchdog", "Failed to load module: " AZ_STRING_FORMAT "\n", AZ_STRING_ARG(nonLoadedModule));
            }
            AZ_Fatal("Watchdog", "Some required modules were not loaded. Check the console output for errors");

            std::terminate();
        }
    }

    AZStd::set<AZStd::string> WatchdogToolsSystemComponent::GatherDynamicModules(const AZStd::vector<AZStd::string>& modules)
    {
        AZStd::set<AZStd::string> results;
        for (auto module : modules)
        {
            auto fileName{ module };
            if (!fileName.starts_with(AZ_TRAIT_OS_DYNAMIC_LIBRARY_PREFIX))
            {
                fileName = AZ_TRAIT_OS_DYNAMIC_LIBRARY_PREFIX + fileName;
            }

            if (!fileName.ends_with(AZ_TRAIT_OS_DYNAMIC_LIBRARY_EXTENSION))
            {
                fileName += AZ_TRAIT_OS_DYNAMIC_LIBRARY_EXTENSION;
            }

            results.insert(fileName);
        }
        return results;
    }
} // namespace WatchdogTools
