
#pragma once

#include <SplineTools/SplineToolsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace SplineTools
{
    class SplineToolsRequests
    {
    public:
        AZ_RTTI(SplineToolsRequests, SplineToolsRequestsTypeId);
        virtual ~SplineToolsRequests() = default;
        // Put your public methods here
    };

    class SplineToolsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SplineToolsRequestBus = AZ::EBus<SplineToolsRequests, SplineToolsBusTraits>;
    using SplineToolsInterface = AZ::Interface<SplineToolsRequests>;

} // namespace SplineTools
