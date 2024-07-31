
#pragma once

#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2ScriptIntegration
{
    class ROS2ScriptIntegrationRequests
    {
    public:
        AZ_RTTI(ROS2ScriptIntegrationRequests, ROS2ScriptIntegrationRequestsTypeId);
        virtual ~ROS2ScriptIntegrationRequests() = default;
        // Put your public methods here
    };

    class ROS2ScriptIntegrationBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2ScriptIntegrationRequestBus = AZ::EBus<ROS2ScriptIntegrationRequests, ROS2ScriptIntegrationBusTraits>;
    using ROS2ScriptIntegrationInterface = AZ::Interface<ROS2ScriptIntegrationRequests>;

} // namespace ROS2ScriptIntegration
