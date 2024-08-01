
#pragma once

#include <DisableMainView/DisableMainViewTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace DisableMainView
{
    class DisableMainViewRequests
    {
    public:
        AZ_RTTI(DisableMainViewRequests, DisableMainViewRequestsTypeId);
        virtual ~DisableMainViewRequests() = default;
        // Put your public methods here
    };

    class DisableMainViewBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using DisableMainViewRequestBus = AZ::EBus<DisableMainViewRequests, DisableMainViewBusTraits>;
    using DisableMainViewInterface = AZ::Interface<DisableMainViewRequests>;

} // namespace DisableMainView
