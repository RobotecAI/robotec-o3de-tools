
#pragma once

#include <ROS2PoseControl/ROS2PoseControlTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2PoseControl
{
    class ROS2PoseControlRequests
    {
    public:
        AZ_RTTI(ROS2PoseControlRequests, ROS2PoseControlRequestsTypeId);
        virtual ~ROS2PoseControlRequests() = default;
        // Put your public methods here
    };

    class ROS2PoseControlBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2PoseControlRequestBus = AZ::EBus<ROS2PoseControlRequests, ROS2PoseControlBusTraits>;
    using ROS2PoseControlInterface = AZ::Interface<ROS2PoseControlRequests>;

} // namespace ROS2PoseControl
