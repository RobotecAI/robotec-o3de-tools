
#pragma once

#include <RandomizeUtils/RandomizeUtilsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RandomizeUtils
{
    class RandomizeUtilsRequests
    {
    public:
        AZ_RTTI(RandomizeUtilsRequests, RandomizeUtilsRequestsTypeId);
        virtual ~RandomizeUtilsRequests() = default;
        // Put your public methods here
    };

    class RandomizeUtilsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RandomizeUtilsRequestBus = AZ::EBus<RandomizeUtilsRequests, RandomizeUtilsBusTraits>;
    using RandomizeUtilsInterface = AZ::Interface<RandomizeUtilsRequests>;

} // namespace RandomizeUtils
