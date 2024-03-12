
#pragma once

#include <RobotecSplineTools/RobotecSplineToolsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RobotecSplineTools
{
    class RobotecSplineToolsRequests
    {
    public:
        AZ_RTTI(RobotecSplineToolsRequests, RobotecSplineToolsRequestsTypeId);
        virtual ~RobotecSplineToolsRequests() = default;
        // Put your public methods here
    };

    class RobotecSplineToolsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RobotecSplineToolsRequestBus = AZ::EBus<RobotecSplineToolsRequests, RobotecSplineToolsBusTraits>;
    using RobotecSplineToolsInterface = AZ::Interface<RobotecSplineToolsRequests>;

} // namespace RobotecSplineTools
