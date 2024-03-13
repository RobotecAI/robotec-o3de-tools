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
        AZ_Trace("Watchdog", "WatchdogTools Component is being activated");

        AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
        AZ_Assert(settingsRegistry, "Settings Registry Interface not available");
        WatchdogSettings watchdogSettings;
        watchdogSettings.LoadSettings(settingsRegistry);
        AZStd::set<AZStd::string> requiredComponents = {};
        requiredComponents.insert(watchdogSettings.m_requiredModules.begin(), watchdogSettings.m_requiredModules.end());
        AZ_Trace("Watchdog", "Count of required modules: %zu", requiredComponents.size());
        AZ::ModuleManagerRequestBus::Broadcast(
            &AZ::ModuleManagerRequestBus::Events::EnumerateModules,
            [&requiredComponents](const AZ::ModuleData& moduleData)
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
                else
                {
                    AZStd::string moduleName = moduleData.GetDebugName();

                    for (auto& r : requiredComponents)
                    {
                        if (moduleName.starts_with(r))
                        {
                            requiredComponents.erase(r);
                            break;
                        }
                    }
                    // AZ_Info("Watchdog", "Module %s is loaded ", moduleData.GetDebugName());
                    return true;
                }

                return true;
            });

        if (!requiredComponents.empty())
        {
            AZ_Fatal("Watchdog", "Some required components were not loaded. Check the console output for errors");
            std::terminate();
            // AZ_Crash();
        }
    }

    void WatchdogToolsSystemComponent::Deactivate()
    {
        WatchdogToolsRequestBus::Handler::BusDisconnect();
    }

} // namespace WatchdogTools
