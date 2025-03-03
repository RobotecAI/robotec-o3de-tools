#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/IO/Path/Path_fwd.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/string/string.h>

namespace FPSProfiler
{
    class FPSProfilerRequests
    {
    public:
        AZ_RTTI(FPSProfilerRequests, FPSProfilerRequestsTypeId);
        virtual ~FPSProfilerRequests() = default;

        // Profiler control
        virtual void StartProfiling() = 0;
        virtual void StopProfiling() = 0;
        virtual void ResetProfilingData() = 0;
        [[nodiscard]] virtual bool IsProfiling() const = 0;
        [[nodiscard]] virtual bool IsAnySaveOptionEnabled() const = 0;
        virtual void ChangeSavePath(
            const AZ::IO::Path& newSavePath) = 0; //!< Caution! This function is not runtime safe. Instead, use @ref SafeChangeSavePath
        virtual void SafeChangeSavePath(const AZ::IO::Path& newSavePath) = 0; //!< Runtime safe path changing. Saves and stops current
                                                                              //!< profiling and changes path afterward.

        // Get Fps Data
        [[nodiscard]] virtual float GetMinFps() const = 0;
        [[nodiscard]] virtual float GetMaxFps() const = 0;
        [[nodiscard]] virtual float GetAvgFps() const = 0;
        [[nodiscard]] virtual float GetCurrentFps() const = 0;

        // Memory usage
        [[nodiscard]] virtual size_t GetCpuMemoryUsed() const = 0;
        [[nodiscard]] virtual size_t GetGpuMemoryUsed() const = 0;

        // Logging
        virtual void SaveLogToFile() = 0;
        virtual void SaveLogToFileWithNewPath(const AZStd::string& newSavePath, bool useSafeChangePath) = 0;
        virtual void ShowFpsOnScreen(bool enable) = 0;
    };

    class FPSProfilerBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using FPSProfilerRequestBus = AZ::EBus<FPSProfilerRequests, FPSProfilerBusTraits>;
    using FPSProfilerInterface = AZ::Interface<FPSProfilerRequests>;

    // Notifications EBus (For Sending Events)
    class FPSProfilerNotifications
    {
    public:
        AZ_RTTI(FPSProfilerNotifications, FPSProfilerNotificationsTypeId);
        virtual ~FPSProfilerNotifications() = default;

        virtual void OnFileCreated(const AZStd::string& fileName)
        {
        }
        virtual void OnFileUpdate(const AZStd::string& fileName)
        {
        }
        virtual void OnFileSaved(const AZStd::string& fileName)
        {
        }
    };

    class FPSProfilerNotificationBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple; // Allow multiple listeners
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single; // Only one global address
        //////////////////////////////////////////////////////////////////////////
    };

    using FPSProfilerNotificationBus = AZ::EBus<FPSProfilerNotifications, FPSProfilerNotificationBusTraits>;

} // namespace FPSProfiler
