
#pragma once

#include <TerrainShaper/TerrainShaperTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace TerrainShaper
{
    class TerrainShaperRequests
    {
    public:
        AZ_RTTI(TerrainShaperRequests, TerrainShaperRequestsTypeId);
        virtual ~TerrainShaperRequests() = default;
        // Put your public methods here
    };
    
    class TerrainShaperBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using TerrainShaperRequestBus = AZ::EBus<TerrainShaperRequests, TerrainShaperBusTraits>;
    using TerrainShaperInterface = AZ::Interface<TerrainShaperRequests>;

} // namespace TerrainShaper
