#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/EBus/EBus.h>
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
        virtual bool IsProfiling() const = 0;

        // Get Fps Data
        virtual float GetMinFps() const = 0;
        virtual float GetMaxFps() const = 0;
        virtual float GetAvgFps() const = 0;
        virtual float GetCurrentFps() const = 0;

        // Memory usage
        virtual size_t GetCpuMemoryUsed() const = 0;
        virtual size_t GetGpuMemoryUsed() const = 0;

        // Logging
        virtual void SaveLogToFile() = 0;
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
