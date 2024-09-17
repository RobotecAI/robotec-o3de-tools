
#pragma once

#include <CsvSpawner/CsvSpawnerTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace CsvSpawner
{
    class CsvSpawnerRequests
    {
    public:
        AZ_RTTI(CsvSpawnerRequests, CsvSpawnerRequestsTypeId);
        virtual ~CsvSpawnerRequests() = default;
        // Put your public methods here
    };

    class CsvSpawnerBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using CsvSpawnerRequestBus = AZ::EBus<CsvSpawnerRequests, CsvSpawnerBusTraits>;
    using CsvSpawnerInterface = AZ::Interface<CsvSpawnerRequests>;

} // namespace CsvSpawner
