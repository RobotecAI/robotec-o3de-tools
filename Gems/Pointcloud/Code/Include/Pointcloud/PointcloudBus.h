
#pragma once

#include <Pointcloud/PointcloudTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace Pointcloud
{
    class PointcloudRequests
    {
    public:
        AZ_RTTI(PointcloudRequests, PointcloudRequestsTypeId);
        virtual ~PointcloudRequests() = default;
        // Put your public methods here
    };

    class PointcloudBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using PointcloudRequestBus = AZ::EBus<PointcloudRequests, PointcloudBusTraits>;
    using PointcloudInterface = AZ::Interface<PointcloudRequests>;

} // namespace Pointcloud
