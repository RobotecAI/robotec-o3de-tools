#pragma once
#include "Utils/Constants.h"
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/string/string.h>

namespace ViewportStreamer::RegistryUtilities
{
    // general function to take value from registry and handling default value
    template<typename T>
    T GetValueFromSettingsRegistry(AZStd::string key, T defaultValue)
    {
        AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
        AZ_Assert(settingsRegistry, "Settings Registry is not available");
        T setregValue;
        auto result = settingsRegistry->Get(setregValue, key);
        return result ? setregValue : defaultValue;
    }

    inline AZ::u64 GetViewportStreamerFrequency()
    {
        return GetValueFromSettingsRegistry(
            AZStd::string(ViewportStreamer::Constants::ViewportStreamerFrequencyRegistryKey),
            ViewportStreamer::Constants::ViewportStreamerFrequency);
    }

    inline AZStd::string GetViewportStreamerFrameName()
    {
        return GetValueFromSettingsRegistry(
            AZStd::string(ViewportStreamer::Constants::ViewportStreamerFrameNameRegistryKey),
            AZStd::string(ViewportStreamer::Constants::ViewportStreamerFrameName));
    }
} // namespace ViewportStreamer::RegistryUtilities
