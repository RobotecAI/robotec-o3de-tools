
#pragma once

#include <RobotecRecordingTools/RobotecRecordingToolsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RobotecRecordingTools
{
    class RobotecRecordingToolsRequests
    {
    public:
        AZ_RTTI(RobotecRecordingToolsRequests, RobotecRecordingToolsRequestsTypeId);
        virtual ~RobotecRecordingToolsRequests() = default;
        // Put your public methods here
    };

    class RobotecRecordingToolsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RobotecRecordingToolsRequestBus = AZ::EBus<RobotecRecordingToolsRequests, RobotecRecordingToolsBusTraits>;
    using RobotecRecordingToolsInterface = AZ::Interface<RobotecRecordingToolsRequests>;

} // namespace RobotecRecordingTools
