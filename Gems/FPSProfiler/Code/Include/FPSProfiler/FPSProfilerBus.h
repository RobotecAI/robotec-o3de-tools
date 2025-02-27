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
        // Put your public methods here
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

        virtual void OnFileCreated(AZStd::string fileName)
        {
        }
        virtual void OnFileUpdate(AZStd::string fileName)
        {
        }
        virtual void OnFileSaved(AZStd::string fileName)
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
