
#pragma once

#include <WatchdogTools/WatchdogToolsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace WatchdogTools
{
    class WatchdogToolsRequests
    {
    public:
        AZ_RTTI(WatchdogToolsRequests, WatchdogToolsRequestsTypeId);
        virtual ~WatchdogToolsRequests() = default;
        // Put your public methods here
    };

    class WatchdogToolsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using WatchdogToolsRequestBus = AZ::EBus<WatchdogToolsRequests, WatchdogToolsBusTraits>;
    using WatchdogToolsInterface = AZ::Interface<WatchdogToolsRequests>;

} // namespace WatchdogTools
